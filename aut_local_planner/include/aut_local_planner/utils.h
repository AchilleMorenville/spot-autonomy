#ifndef AUT_LOCAL_PLANNER_UTILS_H_
#define AUT_LOCAL_PLANNER_UTILS_H_

namespace aut_local_planner {

struct Motion {
	float d_x;
	float d_y;
	float d_theta;

	Motion();
	Motion(const float d_x, const float d_y, const float d_theta);
	Motion(const Motion& m);
	Motion& operator=(Motion m);
	~Motion();

	bool IsStatic();
};

}  // namespace aut_local_planner

#endif  // AUT_LOCAL_PLANNER_UTILS_H_