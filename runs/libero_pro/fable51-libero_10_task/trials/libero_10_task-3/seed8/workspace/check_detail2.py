from check_detail import *
detail(plan['push0'][-1:]+plan['push1']+plan['pushup'],'pushA')
detail([plan['pushup'][-1]]+plan['prepushB']+plan['push0B'],'pushupA->push0B')
detail(plan['push0B'][-1:]+plan['push1B']+plan['pushupB'],'pushB')
# link clearances during pushA
for q in plan['push0'][-1:]+plan['push1']:
    lc=link_clearance(r,q); print(np.round(r.tcp(q)[0],3),[(n,round(c,3)) for n,c,_ in lc if c<0.05])
