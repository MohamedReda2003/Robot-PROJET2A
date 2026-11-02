import cadquery as cq, os, math
out='/mnt/user-data/outputs/STEP_chassis';os.makedirs(out,exist_ok=True)
V=cq.Vector
def bx(x0,y0,z0,dx,dy,dz): return cq.Workplane("XY").box(dx,dy,dz,centered=False).translate((x0,y0,z0))
def cyl(x,y,z,d,h,ax='z'):
    dr={'x':V(1,0,0),'y':V(0,1,0),'z':V(0,0,1)}[ax]
    return cq.Workplane("XY").add(cq.Solid.makeCylinder(d/2,h,V(x,y,z),dr))
def cut(m,*c):
    for k in c: m=m.cut(k)
    return m
def cone_up(x,y,ztop,r): return cq.Workplane("XY").add(cq.Solid.makeCone(0,r,r,V(x,y,ztop-r),V(0,0,1)))
def save(n,m):
    cq.exporters.export(m,f'{out}/{n}.step');v=m.val().Volume()/1000
    print(n,round(v,1),'cm3',m.val().isValid())
def tube(L,a=40,i=34): return cut(bx(0,0,0,L,a,a),bx(-1,(a-i)/2,(a-i)/2,L+2,i,i))
def hY(m,x,z,d=5.6,a=40): return cut(m,cyl(x,-2,z,d,a+4,'y'))
for n,hs in (('P01A_longeron_arriere',[(100,10),(200,10),(210,30)]),('P01B_longeron_avant',[(40,30),(50,10),(150,10)])):
    m=tube(250)
    for x,z in hs: m=hY(m,x,z)
    save(n,m)
m=cut(bx(0,0,0,120,33.4,33.4),bx(-1,4,4,122,25.4,25.4))
for x in (20,100): m=hY(m,x,26.7,5.6,33.4)
save('P02_manchon',m)
m=tube(204)
for x in (20,184): m=hY(m,x,20,4.6)
for x in (47,157): m=cut(m,cyl(x,20,-2,5.6,44))
save('P03_traverse',m)
m=bx(0,0,0,40,40,8).union(bx(3.3,3.3,8,33.4,33.4,40))
hexp=cq.Workplane("XY").workplane(offset=3.5).center(20,10).polygon(6,9.4).extrude(4.6)
m=cut(m,bx(7.3,7.3,8,25.4,25.4,41),cyl(20,10,-1,5.6,12),hexp,cyl(-1,20,28,4.6,60,'x'))
save('P04_embase_traverse',m)
m=bx(0,0,0,100,240,8);c=[]
for x,y in [(10,65),(90,65),(10,175),(90,175)]: c+=[cyl(x,y,-1,5.6,10),cone_up(x,y,8,5.25)]
for x,y in [(20,75),(80,75),(20,165),(80,165)]: c.append(cyl(x,y,-1,7.0,10))
save('P05_plaque_bras',cut(m,*c))
m=bx(0,0,0,100,150,4);c=[]
for x in (10,90): c+=[cyl(x,55,-1,5.6,6),cone_up(x,55,4,5.25)]
save('P06_tuile_plancher',cut(m,*c))
def plate(bolts,acc,axl,slot=None,motor=None):
    m=bx(0,0,0,225,105,8);c=[cyl(x,y,-1,5.6,10) for x,y in bolts]
    c+=[cyl(acc[0],acc[1],-1,12.4,10),cyl(axl[0],axl[1],-1,8.4,10)]
    if slot: c+=[cyl(slot[0]-10,slot[1],-1,8.4,10),cyl(slot[0]+10,slot[1],-1,8.4,10),bx(slot[0]-10,slot[1]-4.2,-1,20,8.4,10)]
    if motor:
        c.append(cyl(motor[0],motor[1],-1,12.4,10))
        for k in range(6): c.append(cyl(motor[0]+15.5*math.cos(k*math.pi/3),motor[1]+15.5*math.sin(k*math.pi/3),-1,3.4,10))
    return cut(m,*c)
save('P07A_plaque_laterale_arriere',plate([(75,70),(175,70)],(185,90),(125,16),motor=(45,35)))
save('P07B_plaque_laterale_avant',plate([(50,70),(150,70)],(40,90),(100,16),slot=(180,35)))
def rev(p): return cq.Workplane("XZ").polyline(p).close().revolve(360,(0,0,0),(0,1,0))
P08=[(3,0),(45,0),(45,3),(39,3),(39,51),(45,57),(45,60),(35,60),(35,6),(10,6),(10,20),(3,20)]
save('P08_roue_motrice',cut(rev(P08),cyl(1.5,0,10,3.4,10,'x')))
P09=[(11.1,0),(45,0),(45,3),(39,3),(39,51),(45,57),(45,60),(35,60),(35,6),(16,6),(16,20),(11.1,20),(11.1,12.8),(4.1,12.8),(4.1,7.2),(11.1,7.2)]
save('P09_roue_libre',rev(P09))
P10=[(11.1,0),(20,0),(20,24),(11.1,24),(11.1,16.8),(4.1,16.8),(4.1,7.2),(11.1,7.2)]
save('P10_galet',rev(P10))
