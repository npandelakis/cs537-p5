#include "param.h"
#include "types.h"
#include "defs.h"
#include "x86.h"
#include "memlayout.h"
#include "mmu.h"
#include "proc.h"
#include "elf.h"
#include "stddef.h"
#include "sys/types.h"
#include "mmap.h"

// TODO: Clean up maybe?
#include "fs.h"
#include "spinlock.h"
#include "sleeplock.h"
#include "file.h"

#define MAX_REGIONS 10

extern char data[];  // defined by kernel.ld
pde_t *kpgdir;  // for use in scheduler()

// Set up CPU's kernel segment descriptors.
// Run once on entry on each CPU.
void
seginit(void)
{
  struct cpu *c;

  // Map "logical" addresses to virtual addresses using identity map.
  // Cannot share a CODE descriptor for both kernel and user
  // because it would have to have DPL_USR, but the CPU forbids
  // an interrupt from CPL=0 to DPL=3.
  c = &cpus[cpuid()];
  c->gdt[SEG_KCODE] = SEG(STA_X|STA_R, 0, 0xffffffff, 0);
  c->gdt[SEG_KDATA] = SEG(STA_W, 0, 0xffffffff, 0);
  c->gdt[SEG_UCODE] = SEG(STA_X|STA_R, 0, 0xffffffff, DPL_USER);
  c->gdt[SEG_UDATA] = SEG(STA_W, 0, 0xffffffff, DPL_USER);
  lgdt(c->gdt, sizeof(c->gdt));
}

// Return the address of the PTE in page table pgdir
// that corresponds to virtual address va.  If alloc!=0,
// create any required page table pages.
static pte_t *
walkpgdir(pde_t *pgdir, const void *va, int alloc)
{
  pde_t *pde;
  pte_t *pgtab;

  pde = &pgdir[PDX(va)];
  if(*pde & PTE_P){
    pgtab = (pte_t*)P2V(PTE_ADDR(*pde));
  } else {
    if(!alloc || (pgtab = (pte_t*)kalloc()) == 0)
      return 0;
    // Make sure all those PTE_P bits are zero.
    memset(pgtab, 0, PGSIZE);
    // The permissions here are overly generous, but they can
    // be further restricted by the permissions in the page table
    // entries, if necessary.
    *pde = V2P(pgtab) | PTE_P | PTE_W | PTE_U;
  }
  return &pgtab[PTX(va)];
}

// Create PTEs for virtual addresses starting at va that refer to
// physical addresses starting at pa. va and size might not
// be page-aligned.
static int
mappages(pde_t *pgdir, void *va, uint size, uint pa, int perm)
{
  char *a, *last;
  pte_t *pte;

  a = (char*)PGROUNDDOWN((uint)va);
  last = (char*)PGROUNDDOWN(((uint)va) + size - 1);
  for(;;){
    if((pte = walkpgdir(pgdir, a, 1)) == 0)
      return -1;
    if(*pte & PTE_P)
      panic("remap");
    *pte = pa | perm | PTE_P;
    if(a == last)
      break;
    a += PGSIZE;
    pa += PGSIZE;
  }
  return 0;
}

// There is one page table per process, plus one that's used when
// a CPU is not running any process (kpgdir). The kernel uses the
// current process's page table during system calls and interrupts;
// page protection bits prevent user code from using the kernel's
// mappings.
//
// setupkvm() and exec() set up every page table like this:
//
//   0..KERNBASE: user memory (text+data+stack+heap), mapped to
//                phys memory allocated by the kernel
//   KERNBASE..KERNBASE+EXTMEM: mapped to 0..EXTMEM (for I/O space)
//   KERNBASE+EXTMEM..data: mapped to EXTMEM..V2P(data)
//                for the kernel's instructions and r/o data
//   data..KERNBASE+PHYSTOP: mapped to V2P(data)..PHYSTOP,
//                                  rw data + free physical memory
//   0xfe000000..0: mapped direct (devices such as ioapic)
//
// The kernel allocates physical memory for its heap and for user memory
// between V2P(end) and the end of physical memory (PHYSTOP)
// (directly addressable from end..P2V(PHYSTOP)).

// This table defines the kernel's mappings, which are present in
// every process's page table.
static struct kmap {
  void *virt;
  uint phys_start;
  uint phys_end;
  int perm;
} kmap[] = {
 { (void*)KERNBASE, 0,             EXTMEM,    PTE_W}, // I/O space
 { (void*)KERNLINK, V2P(KERNLINK), V2P(data), 0},     // kern text+rodata
 { (void*)data,     V2P(data),     PHYSTOP,   PTE_W}, // kern data+memory
 { (void*)DEVSPACE, DEVSPACE,      0,         PTE_W}, // more devices
};

// Set up kernel part of a page table.
pde_t*
setupkvm(void)
{
  pde_t *pgdir;
  struct kmap *k;

  if((pgdir = (pde_t*)kalloc()) == 0)
    return 0;
  memset(pgdir, 0, PGSIZE);
  if (P2V(PHYSTOP) > (void*)DEVSPACE)
    panic("PHYSTOP too high");
  for(k = kmap; k < &kmap[NELEM(kmap)]; k++)
    if(mappages(pgdir, k->virt, k->phys_end - k->phys_start,
                (uint)k->phys_start, k->perm) < 0) {
      freevm(pgdir);
      return 0;
    }
  return pgdir;
}

// Allocate one page table for the machine for the kernel address
// space for scheduler processes.
void
kvmalloc(void)
{
  kpgdir = setupkvm();
  switchkvm();
}

// Switch h/w page table register to the kernel-only page table,
// for when no process is running.
void
switchkvm(void)
{
  lcr3(V2P(kpgdir));   // switch to the kernel page table
}

// Switch TSS and h/w page table to correspond to process p.
void
switchuvm(struct proc *p)
{
  if(p == 0)
    panic("switchuvm: no process");
  if(p->kstack == 0)
    panic("switchuvm: no kstack");
  if(p->pgdir == 0)
    panic("switchuvm: no pgdir");

  pushcli();
  mycpu()->gdt[SEG_TSS] = SEG16(STS_T32A, &mycpu()->ts,
                                sizeof(mycpu()->ts)-1, 0);
  mycpu()->gdt[SEG_TSS].s = 0;
  mycpu()->ts.ss0 = SEG_KDATA << 3;
  mycpu()->ts.esp0 = (uint)p->kstack + KSTACKSIZE;
  // setting IOPL=0 in eflags *and* iomb beyond the tss segment limit
  // forbids I/O instructions (e.g., inb and outb) from user space
  mycpu()->ts.iomb = (ushort) 0xFFFF;
  ltr(SEG_TSS << 3);
  lcr3(V2P(p->pgdir));  // switch to process's address space
  popcli();
}

// Load the initcode into address 0 of pgdir.
// sz must be less than a page.
void
inituvm(pde_t *pgdir, char *init, uint sz)
{
  char *mem;

  if(sz >= PGSIZE)
    panic("inituvm: more than a page");
  mem = kalloc();
  memset(mem, 0, PGSIZE);
  mappages(pgdir, 0, PGSIZE, V2P(mem), PTE_W|PTE_U);
  memmove(mem, init, sz);
}

// Load a program segment into pgdir.  addr must be page-aligned
// and the pages from addr to addr+sz must already be mapped.
int
loaduvm(pde_t *pgdir, char *addr, struct inode *ip, uint offset, uint sz)
{
  uint i, pa, n;
  pte_t *pte;

  if((uint) addr % PGSIZE != 0)
    panic("loaduvm: addr must be page aligned");
  for(i = 0; i < sz; i += PGSIZE){
    if((pte = walkpgdir(pgdir, addr+i, 0)) == 0)
      panic("loaduvm: address should exist");
    pa = PTE_ADDR(*pte);
    if(sz - i < PGSIZE)
      n = sz - i;
    else
      n = PGSIZE;
    if(readi(ip, P2V(pa), offset+i, n) != n)
      return -1;
  }
  return 0;
}

// Allocate page tables and physical memory to grow process from oldsz to
// newsz, which need not be page aligned.  Returns new size or 0 on error.
int
allocuvm(pde_t *pgdir, uint oldsz, uint newsz)
{
  char *mem;
  uint a;

  if(newsz >= KERNBASE)
    return 0;
  if(newsz < oldsz)
    return oldsz;

  a = PGROUNDUP(oldsz);
  for(; a < newsz; a += PGSIZE){
    mem = kalloc();
    if(mem == 0){
      cprintf("allocuvm out of memory\n");
      deallocuvm(pgdir, newsz, oldsz);
      return 0;
    }
    memset(mem, 0, PGSIZE);
    if(mappages(pgdir, (char*)a, PGSIZE, V2P(mem), PTE_W|PTE_U) < 0){
      cprintf("allocuvm out of memory (2)\n");
      deallocuvm(pgdir, newsz, oldsz);
      kfree(mem);
      return 0;
    }
  }
  return newsz;
}

// Deallocate user pages to bring the process size from oldsz to
// newsz.  oldsz and newsz need not be page-aligned, nor does newsz
// need to be less than oldsz.  oldsz can be larger than the actual
// process size.  Returns the new process size.
int
deallocuvm(pde_t *pgdir, uint oldsz, uint newsz)
{
  pte_t *pte;
  uint a, pa;

  if(newsz >= oldsz)
    return oldsz;

  a = PGROUNDUP(newsz);
  for(; a  < oldsz; a += PGSIZE){
    pte = walkpgdir(pgdir, (char*)a, 0);
    if(!pte)
      a = PGADDR(PDX(a) + 1, 0, 0) - PGSIZE;
    else if((*pte & PTE_P) != 0){
      pa = PTE_ADDR(*pte);
      if(pa == 0)
        panic("kfree");
      char *v = P2V(pa);
      kfree(v);
      *pte = 0;
    }
  }
  return newsz;
}

// Free a page table and all the physical memory pages
// in the user part.
void
freevm(pde_t *pgdir)
{
  uint i;

  if(pgdir == 0)
    panic("freevm: no pgdir");
  deallocuvm(pgdir, KERNBASE, 0);
  for(i = 0; i < NPDENTRIES; i++){
    if(pgdir[i] & PTE_P){
      char * v = P2V(PTE_ADDR(pgdir[i]));
      kfree(v);
    }
  }
  kfree((char*)pgdir);
}

// Clear PTE_U on a page. Used to create an inaccessible
// page beneath the user stack.
void
clearpteu(pde_t *pgdir, char *uva)
{
  pte_t *pte;

  pte = walkpgdir(pgdir, uva, 0);
  if(pte == 0)
    panic("clearpteu");
  *pte &= ~PTE_U;
}

// Given a parent process's page table, create a copy
// of it for a child.
pde_t*
copyuvm(pde_t *pgdir, uint sz)
{
  pde_t *d;
  pte_t *pte;
  uint pa, i, flags;
  char *mem;

  if((d = setupkvm()) == 0)
    return 0;
  for(i = 0; i < sz; i += PGSIZE){
    if((pte = walkpgdir(pgdir, (void *) i, 0)) == 0)
      panic("copyuvm: pte should exist");
    if(!(*pte & PTE_P))
      panic("copyuvm: page not present");
    pa = PTE_ADDR(*pte);
    flags = PTE_FLAGS(*pte);
    if((mem = kalloc()) == 0)
      goto bad;
    memmove(mem, (char*)P2V(pa), PGSIZE);
    if(mappages(d, (void*)i, PGSIZE, V2P(mem), flags) < 0) {
      kfree(mem);
      goto bad;
    }
  }
  return d;

bad:
  freevm(d);
  return 0;
}

int
copy_mmap_pgdir(pde_t *pgdir, struct mmap_area *mmapArea) {
  pte_t *pte;
  char *mem;
  cprintf("copy mmap pgdir\n");
  for(uint i=mmapArea->start_addr; i<mmapArea->end_addr; i+=PGSIZE) {
    if((pte = walkpgdir(myproc()->pgdir, (void *)i,0)) == 0) {
      panic("copy_mmap_pgdr: pte should exist");
    }
    if ((mmapArea->flags & MAP_SHARED) == MAP_SHARED) {
      cprintf("map shared");
      if(mappages(pgdir, (void*)i, PGSIZE, PTE_ADDR(*pte), PTE_W|PTE_U) < 0) {
        panic("mappages shared");
      }
    } else {
      cprintf("map shared");
      if(!(*pte & PTE_P))
      panic("copy_mmap_pgdr: page not present");
      uint pa = PTE_ADDR(*pte);
      uint flags = PTE_FLAGS(*pte);
      if((mem = kalloc()) == 0) {
        goto bad;
      }
      memmove(mem, (char*)P2V(pa), PGSIZE);
      if(mappages(pgdir, (void*)i, PGSIZE, V2P(mem), flags) < 0) {
        kfree(mem);
        goto bad;
      }
      
    }
  }
  bad:
    freevm(pgdir);
    return -1; 
}

// pde_t*
// copyuvm(pde_t *pgdir, uint sz)
// {
//   pde_t *d;
//   pte_t *pte;
//   uint pa, i, flags;
//   char *mem;

//   if((d = setupkvm()) == 0)
//     return 0;
//   for(i = 0; i < sz; i += PGSIZE){
//     if((pte = walkpgdir(pgdir, (void *) i, 0)) == 0)
//       panic("copyuvm: pte should exist");
//     if(!(*pte & PTE_P))
//       panic("copyuvm: page not present");
//     pa = PTE_ADDR(*pte);
//     flags = PTE_FLAGS(*pte);

//     int mmap_index = getMemoryRegion(i,PGSIZE);
//     cprintf("i: %p \n",(void *) i);
//     if(mmap_index != -1) {
//       cprintf("flags: %d\n",myproc()->mmap_list[mmap_index]->flags);
//       mem = (void *)-1;
//       if ((myproc()->mmap_list[mmap_index]->flags & MAP_SHARED) == MAP_SHARED) {
//         cprintf("map shared\n");
//         mem = P2V(pa);
//       }
//     } else {
//       cprintf("map priv\n");
//       if((mem = kalloc()) == 0)
//         goto bad;
//       memmove(mem, (char*)P2V(pa), PGSIZE);
//     }
    
    
//     if(mappages(d, (void*)i, PGSIZE, V2P(mem), flags) < 0) {
//       kfree(mem);
//       goto bad;
//     }
//   }
//   return d;

// bad:
//   freevm(d);
//   return 0;
// }

//PAGEBREAK!
// Map user virtual address to kernel address.
char*
uva2ka(pde_t *pgdir, char *uva)
{
  pte_t *pte;

  pte = walkpgdir(pgdir, uva, 0);
  if((*pte & PTE_P) == 0)
    return 0;
  if((*pte & PTE_U) == 0)
    return 0;
  return (char*)P2V(PTE_ADDR(*pte));
}

// Copy len bytes from p to user address va in page table pgdir.
// Most useful when pgdir is not the current page table.
// uva2ka ensures this only works for PTE_U pages.
int
copyout(pde_t *pgdir, uint va, void *p, uint len)
{
  char *buf, *pa0;
  uint n, va0;

  buf = (char*)p;
  while(len > 0){
    va0 = (uint)PGROUNDDOWN(va);
    pa0 = uva2ka(pgdir, (char*)va0);
    if(pa0 == 0)
      return -1;
    n = PGSIZE - (va - va0);
    if(n > len)
      n = len;
    memmove(pa0 + (va - va0), buf, n);
    len -= n;
    buf += n;
    va = va0 + PGSIZE;
  }
  return 0;
}

int memoryRegionAvailable(int addr, size_t length){
  struct proc *p = myproc();
  for (int i = 0; i < 32; i ++){
    if (p->mmap_list[i]->valid == 0) {
      continue;
    }
    //check if beginning of requested address is within an allocated region
    if (addr >=p->mmap_list[i]->start_addr && addr < p->mmap_list[i]->end_addr) {
      return 0; //not available
    }
    // //check if end of requested address is within an allocated region
    if (addr + length > p->mmap_list[i]->start_addr && addr + length <= p->mmap_list[i]->end_addr) {
      return 0; //not availabe
    }
  }
  return 1;
}

int getMemoryRegion(int addr, size_t length) {
  struct proc *p = myproc();
  for (int i = 0; i < 32; i ++){
    if (p->mmap_list[i]->valid == 0) {
      continue;
    }
    //check if beginning of requested address is within an allocated region
    if (addr >=p->mmap_list[i]->start_addr && addr < p->mmap_list[i]->end_addr) {
      return i; 
    }
    // //check if end of requested address is within an allocated region
    if (addr + length > p->mmap_list[i]->start_addr && addr + length <= p->mmap_list[i]->end_addr) {
      return i; 
    }
  }
  return -1;
}

char *kalloc_and_map(void *addr, uint length) {
  char *mem = kalloc(); //call kalloc to allocate memory
  if(mem == 0) {
    kfree(mem);
    cprintf("allocuvm out of memory\n");
    return (void *)-1;
  }
  // clear existing data
  memset(mem, 0, PGSIZE);
  
  cprintf("%p\n",mem);

  if (mappages(myproc()->pgdir, addr, length, V2P(mem), PTE_W|PTE_U) < 0) {
    kfree(mem);
    cprintf("mappages\n");
    return (void *)-1;
  }

  return mem;
}


//implementation of mmap
void *mmap(void *addr, size_t length, int prot, int flags, int fd, off_t offset) {
  struct proc *p = myproc();
  char * mem;
  uint u_addr = (uint) addr;
  struct file *f = p->ofile[fd];

  cprintf("FLAGS: %d\n",flags);

  //check valid fd
  if (fd >= 0) {
    if (!p->ofile[fd]) {
      cprintf("file desecriptor\n");
      return (void *) -1;
    }
    ilock(f->ip);
    if (f->ip->size > PGROUNDUP(length)) {
      cprintf("file length\n");
      return (void *) -1;
    }
    iunlock(f->ip);
  }

  //No fixed address
  if ((flags & MAP_FIXED) !=  8) {
    cprintf("flags and not fixed\n");
    uint a = p->mmap_free_addr;
    cprintf("mmap base: %p", (void *)a);

    int found = 0;
    for(; a < KERNBASE; a += PGSIZE){
      if (memoryRegionAvailable(a, length)){
        u_addr = a;
        addr = (void *) a;
        cprintf("u_addr: %d", u_addr);
        found = 1;
        break;
      }   
    }
    if (!found) {
      cprintf("No available free address\n");
      return (void *) -1;
    }
    p->mmap_free_addr = u_addr + PGROUNDUP(length); //probably need to check that next VA is free
  }

  //Fixed case
  // Check bounds
  if(u_addr < MMAPBASE || u_addr >= KERNBASE) {
    cprintf("bounds check\n");
    return (void *)-1;
  }

  // Verify Page alignment
  if (u_addr % PGSIZE != 0) {
    return (void *)-1;
  }

  if (!memoryRegionAvailable(u_addr, length)){
    cprintf("memory region\n");
    return (void *)-1;
  }

  //map pages
  mem = kalloc_and_map(addr, length);

  if (mem == (void *)-1) {
    return mem;
  }

  if (fd >= 0) {
    cprintf("fileread\n");
    cprintf("f: %p\n", f);
    cprintf("fd: %d\n", fd);
    cprintf("mem: %p\n", (void *)mem);
    if (fileread(f, mem, length) < 0) {
      return (void *) -1;
    }
  }

  //search for an empty slot in mmap list
  int slot = -1;
  for (int i =0; i < 32; i++) {
    if (p->mmap_list[i]->valid != 1) {
      slot = i;
      break;
    }
  }
  //store mapped addresses in our mmap list
  p->mmap_list[slot]->valid = 1;
  p->mmap_list[slot]->start_addr = u_addr;
  p->mmap_list[slot]->end_addr = u_addr + length;
  p->mmap_list[slot]->size = length;
  p->mmap_list[slot]->flags = flags;
  p->mmap_list[slot]->fd = fd;
  p->mmap_list[slot]->fileVA = mem;
  p->mmap_list[slot]->f = f;

  return addr;
}


int munmap(void *addr, size_t length) {
  struct proc *p = myproc();
  uint u_addr = (uint) addr;

  if (addr == NULL || length == 0){
    cprintf("validation \n");
    return -1; //invalid arguments
  }

  // Verify Page alignment
  if (u_addr % PGSIZE != 0) {
    cprintf("page alignment fail\n");
    return -1;
  }

  struct mmap_area *mmapArea = (void *) -1;
  int i;
  for (i=0; i<32; i++){
    if (p->mmap_list[i]->start_addr == u_addr && 
        p->mmap_list[i]->end_addr >= u_addr + length) {
          mmapArea = p->mmap_list[i];
          break;
        }
  }

  if (mmapArea == (void *) -1) {
    cprintf("mmapArea \n");
    return -1;
  }

  //check flags and MAP_SHARED
  if (mmapArea->flags & MAP_SHARED) {
    //stuff
    if (mmapArea->fd >= 0) {
      cprintf("filewrite\n");
      struct file *f = p->ofile[mmapArea->fd];
      //char *addr = uva2ka(p->pgdir, (void *) mmapArea->start_addr);
      cprintf("%p , %p, %d \n",f,mmapArea->fileVA, mmapArea->size);
      // Reset offset to write data to start of file
      f->off = 0;
      filewrite(mmapArea->f, mmapArea->fileVA, length);
    }
  }
  //remove mappings from page table
  pte_t *pte;
  for(int page_base = u_addr; page_base < u_addr + length; page_base += PGSIZE){
    //more stuff - walk through page table entries in the page 
    //starting from page_base and unmap addresses
    pte = walkpgdir(p->pgdir, (void *) page_base, 0);
    if (pte == 0) {
      cprintf("pte\n");
      return -1;
    }
    char *v = P2V(PTE_ADDR(*pte)); // Get the virtual address of the page
    kfree(v);
    *pte = 0;
  }

  //clear mmapArea and mmap_list of addr
  if(mmapArea->start_addr+PGROUNDUP(length) >= mmapArea->end_addr){ //if length is >= mapped address range, clear start and end
    mmapArea->start_addr = 0;
    mmapArea->end_addr = 0; 
  } else {
    mmapArea->start_addr= mmapArea->start_addr + PGROUNDUP(length); //else we just need to reset the start addr
  }
  mmapArea->flags = 0;
  mmapArea->fd = 0;
  p->mmap_list[i]->valid = 0;

  return 0;
}



//PAGEBREAK!
// Blank page.
//PAGEBREAK!
// Blank page.
//PAGEBREAK!
// Blank page.

