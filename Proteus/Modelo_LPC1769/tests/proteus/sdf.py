# Banco de pruebas en Proteus real: arma netlists SDF, corre PROSPICE.EXE por consola y decodifica el
# archivo de particion (sondas digitales). El formato del netlist y del archivo se dedujo por prueba
# con Proteus 8.13; no esta documentado.
import struct, subprocess, os, re
PROSPICE = os.environ.get("PROSPICE", "C:/Program Files (x86)/Labcenter Electronics/Proteus 8 Professional/BIN/PROSPICE.EXE")
HDR = """ISIS SCHEMATIC DESCRIPTION FORMAT 8.0
=====================================
Design:   %s
Doc. no.: <NONE>
Revision: <NONE>
Author:   <NONE>
Created:  16/11/2026
Modified: 16/11/2026

"""
class Ckt:
    def __init__(self, name, tstop, tmax=0.025):
        self.name=name; self.parts=[]; self.nets={}; self.props={'PARTITION':'4ECA9967','SIMTYPE':'TRANSIENT','TSTART':'0','TSTOP':str(tstop),'TMAX':str(tmax)}
    def part(self, ref, dev, val='', **p):
        s='%s,%s,%s'%(ref,dev,val)
        for k,v in p.items():
            v=str(v); k=k.replace('__','(').replace('_9',')')
            s+=',%s=%s'%(k, '"%s"'%v if (',' in v or ' ' in v or chr(92) in v) else v)
        self.parts.append(s); return ref
    def conn(self, net, ref, pin, typ='PS'):
        self.nets.setdefault(net,[]).append('%s,%s,%s'%(ref,typ,pin))
    def probe(self, net, name=None):
        name=name or net
        self.part(name,'DPROBE',PRIMITIVE='PASSIVE'); self.conn(net,name,'*','IP')
    def text(self):
        o=HDR%self.name
        o+='*PROPERTIES,%d\n'%len(self.props)+''.join('%s=%s\n'%kv for kv in sorted(self.props.items()))+'\n*MODELDEFS,%d\n'%len(getattr(self,'models',[]))+''.join(m+'\n' for m in getattr(self,'models',[]))+'\n'
        o+='*PARTLIST,%d\n'%len(self.parts)+'\n'.join(sorted(self.parts))+'\n\n'
        o+='*NETLIST,%d\n'%len(self.nets)
        for i,(n,pins) in enumerate(self.nets.items()):
            if n=='GND': o+='GND,%d,CLASS=POWER\nGND,PR\n'%(len(pins)+1)
            elif n.startswith('$'): o+='%s,%d,CLASS=POWER\n%s,PR\n'%(n[1:],len(pins)+1,n[1:])
            else: o+='#%05d,%d\n'%(i,len(pins))
            o+='\n'.join(pins)+'\n\n'
        return o+'*GATES,0\n'
    def run(self, timeout=900):
        f=self.name+'.SDF'; open(f,'w').write(self.text())
        if os.path.exists('4ECA9967.PDF'): os.remove('4ECA9967.PDF')
        r=subprocess.run([PROSPICE,f],capture_output=True,timeout=timeout)
        out=r.stdout.decode('latin-1')
        self.log=[l for l in out.splitlines() if l.startswith('#')]
        return parse_pdf('4ECA9967.PDF') if os.path.exists('4ECA9967.PDF') else {}
def parse_pdf(path):
    d=open(path,'rb').read()
    ev={}; o=1248; t=0.0
    while o+2<=len(d):
        tag=struct.unpack_from('<H',d,o)[0]
        if tag==0xffff: t=struct.unpack_from('<d',d,o+2)[0]; o+=10
        elif tag==0xfffe: break
        else: ev.setdefault(tag,[]).append((t,struct.unpack_from('<I',d,o+2)[0])); o+=6
    names=[]; q=o+2
    while q<len(d) and d[q] and q+1+d[q]<=len(d) and d[q+1:q+1+d[q]].isascii() and d[q]>=1 and all(32<=x<127 for x in d[q+1:q+1+d[q]]):
        n=d[q]; names.append(d[q+1:q+1+n].decode()); q+=1+n+5
    res={nm:ev.get(i,[]) for i,nm in enumerate(names)}
    return res, d[o:]

def lpc(c, ref, prog, pins, **kw):
    """pins: {nombre_pin: red}"""
    p=dict(MODDLL='LPC1769.DLL',PRIMITIVE='DIGITAL',PROGRAM=os.path.abspath(prog),XTAL='12MHz',POPUP='0'); p.update(kw)
    c.part(ref,'LPCXPRESSO-1769','LPCXPRESSO-1769',**p)
    for pin,net in pins.items(): c.conn(net,ref,pin,'IO')
    c.conn('GND',ref,'GND','PP')
def res(c, ref, val, n1, n2, analog=False):
    c.part(ref,'RES',val,PRIMITIVE='ANALOG' if analog else 'DIGITAL',PRIMTYPE='RESISTOR'); c.conn(n1,ref,'1'); c.conn(n2,ref,'2')
def ishi(st): return (st&3)==3
def edges(tr):
    """lista (t, nivel) solo con cambios de nivel logico"""
    o=[]
    for t,st in tr:
        l=ishi(st)
        if not o or o[-1][1]!=l: o.append((t,l))
    return o
def uart(tr, baud=9600):
    """decodifica 8N1; devuelve [(t_inicio, byte)]"""
    e=edges(tr); bit=1.0/baud; out=[]; i=0
    def level(t):
        l=True
        for tt,ll in e:
            if tt>t: break
            l=ll
        return l
    import bisect
    ts=[x[0] for x in e]
    def lvl(t):
        k=bisect.bisect_right(ts,t)-1
        return e[k][1] if k>=0 else True
    t=0.0; k=0
    while k<len(e):
        tt,l=e[k]
        if tt>=t and not l and (k==0 or e[k-1][1]):
            b=0
            for n in range(8):
                if lvl(tt+bit*(1.5+n)): b|=1<<n
            stop=lvl(tt+bit*9.5)
            out.append((tt,b if stop else -1))
            t=tt+bit*9.5
        k+=1
    return out
def text(tr): return ''.join(chr(b) if b>=0 else '?' for t,b in uart(tr))
def lines(tr):
    """[(t_fin, linea)]"""
    o=[]; cur=''
    for t,b in uart(tr):
        ch=chr(b) if b>=0 else '?'
        if ch=='\n': o.append((t,cur.rstrip('\r'))); cur=''
        else: cur+=ch
    return o
def pulses(tr, t0=0, t1=1e9):
    """[(t_subida, ancho)] de pulsos altos completos en [t0,t1)"""
    e=edges(tr); o=[]
    for (ta,la),(tb,lb) in zip(e,e[1:]):
        if la and not lb and t0<=ta<t1: o.append((ta,tb-ta))
    return o
def level_at(tr,t):
    l=None
    for tt,ll in edges(tr):
        if tt>t: break
        l=ll
    return l
def vsrc(c, ref, volts, net):
    c.part(ref,'VSOURCE',str(volts),PRIMITIVE='ANALOG'); c.conn(net,ref,'+'); c.conn('GND',ref,'-')
def sonar(c, ref, trig, echo, dist=None, **kw):
    p=dict(MODDLL='HCSR04.DLL',PRIMITIVE='DIGITAL',DISTANCE='200',MMPERVOLT='100',DELAY='250u',POPUP='0'); p.update(kw)
    c.part(ref,'HCSR04','HCSR04',**p); c.conn(trig,ref,'TRIG','IP'); c.conn(echo,ref,'ECHO','OP')
    if dist: c.conn(dist,ref,'DIST','IP')
    c.conn('GND',ref,'GND','PP')
def errs(c): return [l for l in c.log if l[:2] in ('#E','#W','#F','#X') or 'registro' in l or 'PCONP' in l or 'Fault' in l or 'LOCKUP' in l]
def tempdac(c, ta, tb, ain, r0='20k', ra='35714', rb='71430', rg='1101.3', cap=True):
    vsrc(c,'V5',5,'v5'); res(c,'RD0',r0,'v5','lm',True); res(c,'RDA',ra,ta,'lm',True); res(c,'RDB',rb,tb,'lm',True); res(c,'RDG',rg,'lm','GND',True)
    res(c,'RF1','1k','lm',ain,True); res(c,'RF2','220k',ain,'GND',True)
    if cap: c.part('CF1','CAP','1u',PRIMITIVE='ANALOG',PRIMTYPE='CAPACITOR'); c.conn(ain,'CF1','1'); c.conn('GND','CF1','2')
