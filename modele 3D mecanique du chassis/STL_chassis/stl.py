import numpy as np, trimesh, os
from trimesh import creation as cr
out='/mnt/user-data/outputs/STL_chassis';os.makedirs(out,exist_ok=True)
def bx(x0,y0,z0,dx,dy,dz):
    m=cr.box(extents=(dx,dy,dz));m.apply_translation((x0+dx/2,y0+dy/2,z0+dz/2));return m
def cy(x,y,z,d,h,ax='z',sec=48):
    m=cr.cylinder(radius=d/2,height=h,sections=sec)
    if ax=='x': m.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2,[0,1,0]))
    if ax=='y': m.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2,[1,0,0]))
    m.apply_translation((x,y,z));return m
def sub(m,*c):
    for k in c: m=m.difference(k,engine='manifold')
    return m
def csk(x,y,ztop,d=5.5,dc=10.5,through=(-1,20)):
    h=cy(x,y,ztop-5,d,ztop+30)  # through hole
    r=dc/2;c=cr.cone(radius=r,height=r,sections=48)
    c.apply_transform(trimesh.transformations.rotation_matrix(np.pi,[1,0,0]));c.apply_translation((x,y,ztop-r+r))  # base up at ztop? fix below
    return h,c
def cone_down(x,y,ztop,r):
    c=cr.cone(radius=r,height=r,sections=48)           # base z=0 apex z=r
    c.apply_transform(trimesh.transformations.rotation_matrix(np.pi,[1,0,0]))  # base z=0, apex z=-r
    c.apply_translation((x,y,ztop));return c
def save(name,m):
    m.merge_vertices();m.fix_normals()
    assert m.is_watertight,name
    m.export(f'{out}/{name}.stl');b=m.bounds[1]-m.bounds[0]
    print(f'{name}: {b.round(1)} vol={m.volume/1000:.1f}cm3 ~{m.volume/1000*1.23:.0f}g')
def tube(L,a=40,i=34):
    return sub(bx(0,0,0,L,a,a),bx(-1,(a-i)/2,(a-i)/2,L+2,i,i))
def hY(m,x,z,d=5.6,a=40): return sub(m,cy(x,a/2,z,d,a+4,'y'))
# P01
for n,hs in (('P01A_longeron_arriere',[(100,10),(200,10),(210,30)]),('P01B_longeron_avant',[(40,30),(50,10),(150,10)])):
    m=tube(250)
    for x,z in hs: m=hY(m,x,z)
    save(n,m)
# P02
m=sub(bx(0,0,0,120,33.4,33.4),bx(-1,4,4,122,25.4,25.4))
for x in (20,100): m=hY(m,x,26.7,5.6,33.4)
save('P02_manchon',m)
# P03
m=tube(204)
for x in (20,184): m=hY(m,x,20,4.6)
for x in (47,157): m=sub(m,cy(x,20,20,5.6,44))
save('P03_traverse',m)
# P04
m=bx(0,0,0,40,40,8).union(bx(3.3,3.3,8,33.4,33.4,40),engine='manifold')
m=sub(m,bx(7.3,7.3,8,25.4,25.4,41),cy(20,10,4,5.6,12),cy(20,10,5.75,9.4,4.5,sec=6),cy(20,20,28,4.6,60,'x'))
save('P04_embase_traverse',m)
# P05
m=bx(0,0,0,100,240,8)
cut=[]
for x,y in [(10,65),(90,65),(10,175),(90,175)]: cut+= [cy(x,y,4,5.6,12),cone_down(x,y,8,5.25)]
for x,y in [(20,75),(80,75),(20,165),(80,165)]: cut.append(cy(x,y,4,7.0,12))
save('P05_plaque_bras',sub(m,*cut))
# P06
m=bx(0,0,0,100,150,4);cut=[]
for x in (10,90): cut+=[cy(x,55,2,5.6,8),cone_down(x,55,4,5.25)]
save('P06_tuile_plancher',sub(m,*cut))
# P07
def plate(bolts,acc,axl,slot=None,motor=None):
    m=bx(0,0,0,225,105,8);cut=[]
    for x,y in bolts: cut.append(cy(x,y,4,5.6,12))
    cut.append(cy(*acc,4,12.4,12)) if False else cut.append(cy(acc[0],acc[1],4,12.4,12))
    cut.append(cy(axl[0],axl[1],4,8.4,12))
    if slot: cut+= [cy(slot[0]-10,slot[1],4,8.4,12),cy(slot[0]+10,slot[1],4,8.4,12),bx(slot[0]-10,slot[1]-4.2,-1,20,8.4,10)]
    if motor:
        cut.append(cy(motor[0],motor[1],4,12.4,12))
        for k in range(6): cut.append(cy(motor[0]+15.5*np.cos(k*np.pi/3),motor[1]+15.5*np.sin(k*np.pi/3),4,3.4,12))
    return sub(m,*cut)
save('P07A_plaque_laterale_arriere',plate([(75,70),(175,70)],(185,90),(125,16),motor=(45,35)))
save('P07B_plaque_laterale_avant',plate([(50,70),(150,70)],(40,90),(100,16),slot=(180,35)))
# roues (axe Z, âme sur plateau)
def rev(pts):
    m=cr.revolve(np.array(list(pts)+[pts[0]],float),sections=96);m.merge_vertices();trimesh.repair.fix_normals(m);trimesh.repair.fix_inversion(m);return m
P08=[(3,0),(45,0),(45,3),(39,3),(39,51),(45,57),(45,60),(35,60),(35,6),(10,6),(10,20),(3,20)]
m=rev(P08);m=sub(m,cy(0,0,0,0,0) if False else cy(6.5,0,10,3.4,10,'x'))
save('P08_roue_motrice',m)
P09=[(11.1,0),(45,0),(45,3),(39,3),(39,51),(45,57),(45,60),(35,60),(35,6),(16,6),(16,20),(11.1,20),(11.1,12.8),(4.1,12.8),(4.1,7.2),(11.1,7.2)]
save('P09_roue_libre',rev(P09))
P10=[(11.1,0),(20,0),(20,24),(11.1,24),(11.1,16.8),(4.1,16.8),(4.1,7.2),(11.1,7.2)]
save('P10_galet',rev(P10))
