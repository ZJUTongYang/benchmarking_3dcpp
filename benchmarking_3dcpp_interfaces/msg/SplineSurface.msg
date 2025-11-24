std_msgs/Header header

uint8 spline_type
#   BEZIER=0
#   BSPLINE=1
#   NURBS=2

uint32 u_degree # the degree of polynomials in the u direction
uint32 v_degree # the degree of polynomials in the v direction

uint32 num_u_ctrl_pts # the number of control points in the u direction
uint32 num_v_ctrl_pts # the number of control points in the v direction

ControlPoint[] control_points

float64[] u_knot_vector
float64[] v_knot_vector