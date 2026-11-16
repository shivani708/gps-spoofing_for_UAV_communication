#!/usr/bin/env python3
"""
gen_trajectory.py — Generate gpssim user motion file (lat/lon/alt)
Space-separated, 10 rows/second (gpssim 0.1s time step).
Includes a static hold period before drift begins so the receiver
acquires a 3D fix before the position starts moving.
"""
import argparse, math

def lla_to_ecef(lat, lon, alt):
    a=6378137.0; e2=6.6943799901377997e-3
    la=math.radians(lat); lo=math.radians(lon)
    N=a/math.sqrt(1-e2*math.sin(la)**2)
    return ((N+alt)*math.cos(la)*math.cos(lo),
            (N+alt)*math.cos(la)*math.sin(lo),
            (N*(1-e2)+alt)*math.sin(la))

def ecef_to_lla(x,y,z):
    a=6378137.0; e2=6.6943799901377997e-3
    lon=math.atan2(y,x); p=math.sqrt(x**2+y**2)
    lat=math.atan2(z,p*(1-e2))
    for _ in range(10):
        N=a/math.sqrt(1-e2*math.sin(lat)**2)
        lat=math.atan2(z+e2*N*math.sin(lat),p)
    N=a/math.sqrt(1-e2*math.sin(lat)**2)
    return math.degrees(lat),math.degrees(lon),p/math.cos(lat)-N

ap=argparse.ArgumentParser()
ap.add_argument('--start',    required=True)
ap.add_argument('--target',   required=True)
ap.add_argument('--speed',    type=float, default=2.0)
ap.add_argument('--hold',     type=int,   default=60, help='static seconds before drift')
ap.add_argument('--duration', type=int,   default=300)
ap.add_argument('--out',      default='trajectory.csv')
args=ap.parse_args()

slat,slon,salt=[float(v) for v in args.start.split(',')]
tlat,tlon,talt=[float(v) for v in args.target.split(',')]
sx,sy,sz=lla_to_ecef(slat,slon,salt)
tx,ty,tz=lla_to_ecef(tlat,tlon,talt)
dist=math.sqrt((tx-sx)**2+(ty-sy)**2+(tz-sz)**2)

# gpssim uses 0.1-second time steps → 10 rows per second
ROWS_PER_SEC=10
total_rows=args.duration*ROWS_PER_SEC

print(f"Start  : {slat:.6f}N  {slon:.6f}E  {salt:.1f}m")
print(f"Target : {tlat:.6f}N  {tlon:.6f}E  {talt:.1f}m")
print(f"Dist   : {dist:.1f} m   Speed: {args.speed} m/s   ETA: {dist/args.speed:.0f} s")
print(f"Hold   : {args.hold} s static  →  drift starts at t={args.hold}s")
print(f"Rows   : {total_rows} (10/s for gpssim 0.1s step, SPACE-separated)")

with open(args.out,'w') as f:
    for i in range(total_rows):
        t=i/ROWS_PER_SEC
        ds=t-args.hold  # seconds since drift started
        if dist<1 or ds<=0:
            lat,lon,alt=slat,slon,salt
        else:
            moved=ds*args.speed
            if moved>=dist:
                lat,lon,alt=tlat,tlon,talt
            else:
                frac=moved/dist
                lat,lon,alt=ecef_to_lla(sx+frac*(tx-sx),sy+frac*(ty-sy),sz+frac*(tz-sz))
        # MUST be space-separated — commas cause silent gpssim parse failure
        f.write(f"{lat:.8f} {lon:.8f} {alt:.2f}\n")

print(f"Written {total_rows} rows → {args.out}")
