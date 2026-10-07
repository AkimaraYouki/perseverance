# Tap test: IMU gyro (shared memory, serial-arrival stamp) vs wheel joint speed (CAN 0x29, socket recv stamp).
import mmap,struct,time,socket,yaml,math,json
cfg=yaml.safe_load(open('/home/parksudo/gen2_ws/src/gen2_hardware/config/motors.yaml'))['/**']['ros__parameters']['motors']
wid={cfg[n]['can_id']:(n,cfg[n]['direction']) for n in ('wheel_l','wheel_r')}
F=open('/dev/shm/gen2_imu','rb'); m=mmap.mmap(F.fileno(),0,access=mmap.ACCESS_READ)
s=socket.socket(socket.AF_CAN,socket.SOCK_RAW,socket.CAN_RAW); s.bind(('can0',)); s.setblocking(False)
G=[];W=[];last=-1;t_end=time.monotonic()+float(__import__('sys').argv[1])
while time.monotonic()<t_end:
    seq=struct.unpack_from('<I',m,0)[0]
    if seq!=last and not seq&1:
        rx,cnt=struct.unpack_from('<qq',m,8); gy=struct.unpack_from('<3d',m,8+16+32)
        if struct.unpack_from('<I',m,0)[0]==seq: G.append((rx/1e9,gy[1])); last=seq
    try:
        while True:
            fr=s.recv(16); cid,dlc,d=struct.unpack('<IB3x8s',fr); cid&=0x1FFFFFFF
            if cid>>8==0x29 and (cid&255) in wid:
                v=struct.unpack('>h',d[2:4])[0]*10; n,dr=wid[cid&255]
                W.append((time.monotonic(),n,dr*v/14/10*2*math.pi/60))
    except BlockingIOError: pass
    time.sleep(0.0003)
json.dump(dict(G=G,W=W),open('/home/parksudo/.claude/jobs/10169291/tmp/tap.json','w'))
print('samples gyro',len(G),'wheel',len(W))
