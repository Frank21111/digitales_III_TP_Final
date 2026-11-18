import sys,time,pickle,collections; sys.path.insert(0,sys.argv[1])
from sdf import *
ok=[]
def chk(name,cond,info=''):
    ok.append((name,bool(cond))); print(('PASA ' if cond else 'FALLA'),name,info)
def cnt(r): return collections.Counter(l for t,l in lines(r['tx']))
# 1
c=Ckt('p1',2.2); lpc(c,'U1','01_blinky.axf',{'P0.22':'led'}); res(c,'R1','1k','led','GND'); c.probe('led'); r,_=c.run()
e=edges(r['led']); chk('1 blinky',[round(t,3) for t,l in e[1:]]==[0.5,1.0,1.5,2.0] and not errs(c),e[1:3])
# 2
c=Ckt('p2',1.2); lpc(c,'U1','02_uart_eco.axf',{'P0.2':'tx','P0.3':'rx'}); lpc(c,'U2','probador2.axf',{'P0.2':'rx'}); c.probe('tx'); r,_=c.run()
chk('2 uart',text(r['tx'])=='LPC1769 en Proteus: UART0 OK\r\nhola\r\r\nlineas=1\r\nx\n\r\nlineas=2\r\n' and not errs(c))
# 3 con el HC-SR04 real servido por LPC1769.DLL, DIST por tension
for dv,exp in ((None,'ancho=1166 us d=200 mm'),(2.0,'ancho=1166 us d=200 mm'),(0.85,'ancho=495 us d=85 mm')):
    c=Ckt('p3',0.95); lpc(c,'U1','03_captura.axf',{'P0.2':'tx','P0.5':'trig','P0.4':'echo'}); sonar(c,'S1','trig','echo','dist' if dv else None,MODDLL='LPC1769.DLL')
    if dv: vsrc(c,'V1',dv,'dist')
    c.probe('tx'); c.probe('trig'); r,_=c.run(); k=cnt(r); w=[x[1]*1e6 for x in pulses(r['trig'],0.05)]
    chk('3 captura DIST=%s'%dv,k[exp]>=7 and not errs(c) and 10<=min(w) and max(w)<=13,dict(k))
# 4
c=Ckt('p4',5.3); lpc(c,'U1','04_pwm.axf',{'P0.2':'tx','P2.0':'pwm'}); res(c,'R1','10k','pwm','GND'); c.probe('tx'); c.probe('pwm'); r,_=c.run()
w=[]
for k in range(5):
    p=pulses(r['pwm'],k+0.1,k+0.9); w.append(sorted(set(round(x[1]*1e6) for x in p)))
chk('4 pwm',w==[[],[250],[500],[750],[1000]] and not errs(c),w)
# 5
for v,exp,led in ((0.25,'raw=310 mV=249',False),(2.0,'raw=2482 mV=2000',True)):
    c=Ckt('p5',0.65); lpc(c,'U1','05_adc.axf',{'P0.2':'tx','P0.23':'ain','P0.22':'led'}); vsrc(c,'V1',v,'ain'); res(c,'R1','1k','led','GND'); c.probe('tx'); c.probe('led'); r,_=c.run()
    chk('5 adc %.2f V'%v,cnt(r)[exp]>=2 and edges(r['led'])[-1][1]==led and not errs(c),dict(cnt(r)))
# 6
c=Ckt('p6',1.0); lpc(c,'U1','06_eint.axf',{'P0.2':'tx','P2.10':'btn','P0.22':'led'}); lpc(c,'U2','probador6.axf',{'P2.6':'btn'}); res(c,'R1','1k','led','GND'); c.probe('tx'); c.probe('led'); r,_=c.run()
chk('6 eint',[l for t,l in lines(r['tx'])]==['Prueba EINT0','EINT0 1','EINT0 2','EINT0 3'] and len(edges(r['led']))==4 and not errs(c))
# 7
c=Ckt('p7',1.25); lpc(c,'U1','07_adc_mat.axf',{'P0.2':'tx','P0.23':'ain'}); vsrc(c,'V1',0.25,'ain'); c.probe('tx'); r,_=c.run()
chk('7 adc_mat',cnt(r)['raw=310']==12 and not errs(c),dict(cnt(r)))
# TPI con HC-SR04 real (misma DLL), DAC en DIST, AND para cortar ECHO y divisor 1k/2k
c=Ckt('tpi',23.0)
lpc(c,'U1',sys.argv[2],{'P0.2':'tx','P0.3':'rx','P0.5':'trig','P0.4':'echo','P0.23':'ain','P2.0':'pwm','P2.2':'fan','P0.22':'led','P2.3':'buz'})
lpc(c,'U2','probador9.axf',{'P0.2':'rx','P2.1':'ta','P2.2':'tb','P2.3':'da','P2.4':'db','P2.5':'en'})
sonar(c,'S1','trig','e0','dist',MODDLL='LPC1769.DLL')
c.part('G1','AND_2',PRIMITIVE='DIGITAL'); c.conn('e0','G1','D0','IP'); c.conn('en','G1','D1','IP'); c.conn('e1','G1','Q','OP')
if len(sys.argv)>3 and sys.argv[3]=='divisor':
    res(c,'RE1','1k','e1','echo',True); res(c,'RE2','2k','echo','GND',True)
else:
    c.nets['echo']=c.nets.get('echo',[])+c.nets.pop('e1')
vsrc(c,'V5',5,'v5'); res(c,'RD0','20k','v5','ain',True); res(c,'RDA','35714','ta','ain',True); res(c,'RDB','71430','tb','ain',True); res(c,'RDG','1101.3','ain','GND',True); res(c,'RF2','220k','ain','GND',True)
c.part('CF1','CAP','1u',PRIMITIVE='ANALOG',PRIMTYPE='CAPACITOR'); c.conn('ain','CF1','1'); c.conn('GND','CF1','2')
res(c,'RX0','12.5k','v5','dist',True); res(c,'RXA','7142.9','da','dist',True); res(c,'RXB','5555.6','db','dist',True); res(c,'RXG','1666.7','dist','GND',True)
for n in ('pwm','fan','led','buz'): res(c,'RL'+n,'10k',n,'GND')
for n in ('tx','rx','trig','e0','pwm','fan','led','buz'): c.probe(n)
t0=time.time(); r,_=c.run(timeout=1800); print('TPI wall %.0f s'%(time.time()-t0),errs(c)); pickle.dump(r,open('tpi.pkl','wb'))
L=lines(r['tx'])
def at(t):
    x=[l for tt,l in L if tt<=t]
    return x[-1] if x else None
E={ '1 arranque':(1.9,'L=20,SP=0,T=24.9,TON=45,TOFF=40,PWM=0,FAN=0,ALM=0,ERR=0,MODO=A'),
    '3 SP=120':(3.9,'L=20,SP=120,T=24.9,TON=45,TOFF=40,PWM=100,FAN=0,ALM=0,ERR=0,MODO=A'),
    '4 nivel 110':(5.9,'L=110,SP=120,T=24.9,TON=45,TOFF=40,PWM=15,FAN=0,ALM=0,ERR=0,MODO=A'),
    '5 46 C':(8.4,'L=110,SP=120,T=45.7,TON=45,TOFF=40,PWM=16,FAN=1,ALM=1,ERR=0,MODO=A'),
    '6 39 C':(10.9,'L=110,SP=120,T=38.8,TON=45,TOFF=40,PWM=16,FAN=0,ALM=0,ERR=0,MODO=A'),
    '7 nivel 180':(12.9,'L=180,SP=120,T=38.8,TON=45,TOFF=40,PWM=0,FAN=0,ALM=0,ERR=0,MODO=A'),
    '8 sin eco':(14.9,'L=180,SP=120,T=38.8,TON=45,TOFF=40,PWM=0,FAN=0,ALM=0,ERR=1,MODO=A'),
    '9a TON=35 ignorado':(15.9,'L=180,SP=120,T=38.8,TON=45,TOFF=40,PWM=0,FAN=0,ALM=0,ERR=1,MODO=A'),
    '9b TOFF=30,TON=35':(18.9,'L=180,SP=120,T=38.8,TON=35,TOFF=30,PWM=0,FAN=1,ALM=1,ERR=1,MODO=A'),
    'x fuera de rango':(19.9,'L=180,SP=120,T=38.8,TON=35,TOFF=30,PWM=0,FAN=1,ALM=1,ERR=0,MODO=A'),
    'x U=40':(21.4,'L=20,SP=120,T=38.8,TON=35,TOFF=30,PWM=40,FAN=1,ALM=1,ERR=0,MODO=M'),
    'x SP=0':(22.9,'L=20,SP=0,T=38.8,TON=35,TOFF=30,PWM=0,FAN=1,ALM=1,ERR=0,MODO=A')}
for k,(t,exp) in E.items(): chk('TPI '+k,at(t)==exp,'' if at(t)==exp else at(t))
tp=pulses(r['trig'],0.05); w=[x[1]*1e6 for x in tp]; per=[b[0]-a[0] for a,b in zip(tp,tp[1:])]
chk('TPI 2 TRIG',10<min(w) and max(w)<12.1 and abs(min(per)-0.1)<1e-6 and abs(max(per)-0.1)<1e-6,'%.2f..%.2f us'%(min(w),max(w)))
def pw(a,b): return sorted(set(round(x[1]*1e6) for x in pulses(r['pwm'],a,b)))
chk('TPI PWM',level_at(r['pwm'],3.5)==True and pw(2.3,3.9)==[] and pw(4.6,5.9)==[150] and pw(9.7,10.9)==[160] and pw(11.4,12.9)==[] and pw(20.7,21.4)==[400],(pw(4.6,5.9),pw(9.7,10.9),pw(20.7,21.4)))
for n in ('fan','led','buz'): chk('TPI '+n,[(round(t),l) for t,l in edges(r[n])[1:]]==[(7,True),(9,False),(18,True)],[(round(t,3),l) for t,l in edges(r[n])[1:]])
print('TOTAL %d/%d'%(sum(v for k,v in ok),len(ok)))
