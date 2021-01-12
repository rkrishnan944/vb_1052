#!/usr/bin/env python

def sim_score(arg_sim_time):
	score = (50) * ( (100.0 - arg_sim_time)/100.0 )
	
	if (score < 0):
		return 0

	return score


def main():

	r = raw_input('1. Are you able to place the Red-Package in the Red-Bin? (y/n) -> ')
	if(r == 'y'):
		r_pnt = 5
	else:
		r_pnt = 0


	g = raw_input('2. Are you able to place the Green-Package in the Green-Bin? (y/n) -> ')
	if(g == 'y'):
		g_pnt = 10
	else:
		g_pnt = 0


	b = raw_input('3. Are you able to place the Blue-Package in the Blue-Bin? (y/n) -> ')
	if(b == 'y'):
		b_pnt = 20
	else:
		b_pnt = 0


	coll = raw_input('4. Are you able able to avoid any kind of collision? (y/n) -> ')
	if(r == 'y'):
		coll_pnt = 15
	else:
		coll_pnt = 0


	no_coll = raw_input('5. Total number of collision? (0 or +ve integer) -> ')
	no_coll_pnt = int(no_coll) * (-10)


	sim = raw_input('6. What is your simulation time? (+ve integer) -> ')
	sim_pnt = sim_score( int(sim) )


	t3_score = r_pnt + g_pnt + b_pnt + coll_pnt + no_coll_pnt + sim_pnt

	if(t3_score < 0):
		t3_score = 0

	print("\n\n Task-3 Self-Eval Score: {}".format(str(t3_score)) )

if __name__ == '__main__':
	main()