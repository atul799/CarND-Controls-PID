# -*- coding: utf-8 -*-
"""
This Script use the output file cte.out and plots cte/d_error/i_error
Works only for non twiddle mode
Can be enhanced to capture "Reset" as key word to split data in twiddle mode
to capture data from each twiddle mode 
and plot subsequest twiddle step

@author: atpandey

"""

import matplotlib.pyplot as plt

#%%
out_file='./outputs/cte.out'

Kp=''
Ki=''
Kd=''
#angle,cte,d_error,i_error
angle_list=[]
cte_list=[]
d_error_list=[]
i_error_list=[]

entries=0
for line in  open(out_file):
    line=line.rstrip('\n')
    line_sp=line.split(',')
    line_sp_l=line_sp[:-1]
    #print(line_sp_l)
    
    if(entries>0):
        Kp=float(line_sp_l[2])
        Ki=float(line_sp_l[3])
        Kd=float(line_sp_l[4])
        
        angle_list.append(float(line_sp_l[5]))
        cte_list.append(float(line_sp_l[6]))
        d_error_list.append(float(line_sp_l[7]))
        i_error_list.append(float(line_sp_l[8]))
    
    entries +=1
    
print("Total Number of entries are:",entries-1)  

x=range(entries-1)


fig=plt.figure(figsize=(10, 25))
plt.rcParams['axes.grid'] = True

ax0 = fig.add_subplot(411)
ax1 = fig.add_subplot(412)
ax2 = fig.add_subplot(413)
ax3 = fig.add_subplot(414)

ax0.set_title('angles')
ax1.set_title('cte/p_error')
ax2.set_title('d_error')
ax3.set_title('i_error')


ax0.plot(x, angle_list, linewidth=3)
ax1.plot(x,cte_list , linewidth=3)
ax2.plot(x,d_error_list , linewidth=3)
ax3.plot(x,i_error_list , linewidth=3)
plt.savefig('./outputs/output.png')  
    

