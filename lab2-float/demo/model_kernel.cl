
__kernel void go(__global double *agentX,								
                   __global double *agentY,                                
                   __global double *waypointX,                             
				 __global double *waypointY,                             
				 __global double *waypointR,                             
				 __global bool *waypointRad) {                           
  int i = get_global_id(0);                                              
  double waypointXn = waypointX[i] - agentX[i];                          
  double waypointYn = waypointY[i] - agentY[i];                          
  double length = sqrt(waypointXn*waypointXn + waypointYn*waypointYn);   
  if(length < waypointR[i]) {                                            
	waypointRad[i] = true;                                             
  } else {                                                               
	waypointRad[i] = false;                                            
  }                                                                      
  waypointXn /= length;                                                  
  waypointYn /= length;                                                  
  agentX[i] = round(agentX[i] + waypointXn);                             
  agentY[i] = round(agentY[i] + waypointYn);                             
}
