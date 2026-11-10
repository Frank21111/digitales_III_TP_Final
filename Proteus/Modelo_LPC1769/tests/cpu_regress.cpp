// Regresión de defectos del núcleo encontrados en la revisión: IT tras retorno de excepción,
// prioridad en caché después de SLEEPONEXIT y bits de BusFault precisa.
#include "../src/core/cpu.hpp"
#include <cstdio>
#include <cstring>
using namespace lpcsim;
struct Io : CpuIo {
  bool io_read(uint32_t, int, uint32_t &v) override { v = 0; return false; }
  bool io_write(uint32_t, int, uint32_t) override { return false; }
  void cpu_log(const std::string &m) override { printf("  [log] %s\n", m.c_str()); }
};
static uint8_t code[0x1000], ram[0x2000];
static void w16(uint32_t a, uint16_t v){ memcpy(code+a,&v,2);} 
static void w32(uint32_t a, uint32_t v){ memcpy(code+a,&v,4);} 
static int fails = 0;
int main(){
  // ---- Test 1: IT block + conditional exception return
  {
  memset(code,0,sizeof code); memset(ram,0,sizeof ram);
  w32(0,0x20002000); w32(4,0x101); w32(16*4,0x201); w32(17*4,0x301);
  w16(0x100,0x2000); // movs r0,#0 (Z=1)
  w16(0x102,0xBF0C); // ite eq
  w16(0x104,0x2101); // moveq r1,#1
  w16(0x106,0x2102); // movne r1,#2
  w16(0x108,0xE7FE); // b .
  w16(0x200,0x4280); // cmp r0,r0
  w16(0x202,0xBF08); // it eq
  w16(0x204,0x4770); // bxeq lr
  Io io; Cpu c(&io); c.add_region(0,sizeof code,code,false); c.add_region(0x20000000,sizeof ram,ram,true); c.reset();
  c.scs_write(0xE000E100,4,1);
  c.run(1); c.run(1); // movs, ite
  c.set_pending(16);
  for(int i=0;i<20 && c.pc!=0x108;i++) c.run(1);
  printf("T1 IT+return: r1=%u (expected 1) pc=%08X\n", c.r[1], c.pc); if (c.r[1] != 1) fails++;
  }
  // ---- Test 2: SLEEPONEXIT + can_take_exception stale cached_exec
  {
  memset(code,0,sizeof code); memset(ram,0,sizeof ram);
  w32(0,0x20002000); w32(4,0x101); w32(16*4,0x201); w32(17*4,0x301);
  w16(0x100,0xE7FE); w16(0x200,0x4770); w16(0x300,0xE7FE);
  Io io; Cpu c(&io); c.add_region(0,sizeof code,code,false); c.add_region(0x20000000,sizeof ram,ram,true); c.reset();
  c.scs_write(0xE000E100,4,3); c.scs_write(0xE000E400,1,0x00); c.scs_write(0xE000E401,1,0x80);
  c.scs_write(0xE000ED10,4,2);
  c.set_pending(16); c.set_pending(17);
  c.run(1); c.run(1);
  printf("T2 after return: sleeping=%d pc=%08X\n", c.sleeping, c.pc);
  if (c.sleeping && c.can_take_exception()) c.sleeping=false;
  c.run(200);
  printf("T2 IRQ1 taken? pc=%08X ipsr=%u (expected pc=00000300 ipsr=17)\n", c.pc, c.ipsr); if (c.pc != 0x300 || c.ipsr != 17) fails++;
  }
  // ---- Test 3: data BusFault CFSR bits
  {
  memset(code,0,sizeof code); memset(ram,0,sizeof ram);
  w32(0,0x20002000); w32(4,0x101); w32(3*4,0x401);
  w16(0x100,0x2005); // movs r0,#5
  w16(0x102,0x0700); // lsls r0,r0,#28 -> 0x50000000
  w16(0x104,0x6801); // ldr r1,[r0]
  w16(0x400,0xE7FE);
  Io io; Cpu c(&io); c.add_region(0,sizeof code,code,false); c.add_region(0x20000000,sizeof ram,ram,true); c.reset();
  for(int i=0;i<10;i++) c.run(1);
  uint32_t cfsr; c.scs_read(0xE000ED28,4,cfsr);
  printf("T3 CFSR=%08X (expected PRECISERR|BFARVALID = 00008200)\n", cfsr); if (cfsr != 0x8200) fails++;
  }
  printf("RESULTADO: %d fallas\n", fails);
  return fails ? 1 : 0;
}
