#ifndef PORTAL_MANAGER_H_
#define PORTAL_MANAGER_H_

#include <iostream>
#include <Eigen/Dense>
#include <string>
#include <vector>

#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/collision_object.h>
#include <fcl/narrowphase/distance.h>

#include <fcl/narrowphase/continuous_collision.h>
#include <fcl/narrowphase/continuous_collision_object.h>


#include <Portal.h>


using namespace std;

namespace PortalCrane {
	class Portal_Manager {
	private:
		string hint = "[Portal_Mannager]:";
		vector<shared_ptr<PortalCrane::Portal>> portals;
		vector<vector<int>> check_index;
		vector<vector<double>> distance;
	public:
		Portal_Manager(vector<shared_ptr<PortalCrane::Portal>>& portal);
		~Portal_Manager();
		void cal_distance();
		void cal_distance(double angle1, double angle2);
	private:
		void check_near();
		double cal_twoPortal(shared_ptr<PortalCrane::Portal>& p0, shared_ptr<PortalCrane::Portal>& p1);
		inline double get_distance(const Eigen::Vector2d& p0, const Eigen::Vector2d& p1);
	};
	inline double Portal_Manager::get_distance(const Eigen::Vector2d& p0, const Eigen::Vector2d& p1) {
		Eigen::Vector2d dis = p0 - p1;
		return dis.norm();
	}
}

#endif