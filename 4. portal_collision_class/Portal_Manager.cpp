#include <Portal_Manager.h>


namespace PortalCrane {
	Portal_Manager::Portal_Manager(vector<shared_ptr<PortalCrane::Portal>>& portal):
		portals(portal)
	{
		this->check_near();
		this->cal_distance();
	}

	Portal_Manager::~Portal_Manager()
	{ 
	}

	void Portal_Manager::check_near() {
		for (int i = 0; i < this->portals.size()-1; ++i) { 
			shared_ptr<PortalCrane::Portal> portal1 = this->portals.at(i);
			vector<int> index;
			cout << this->hint << "[";
			for (int j = i+1; j < this->portals.size(); ++j) {
				shared_ptr<PortalCrane::Portal> portal2 = this->portals.at(j);
				
				if (this->get_distance(portal1->base_position, portal2->base_position) < portal1->R_max + portal2->R_max) {
					cout << j;
					index.push_back(j);
				}
				this->check_index.push_back(index);
				if(j != this->portals.size()-1)
					cout << " ";
			}
			cout << "]" << endl;  
		} 
	}

	void Portal_Manager::cal_distance() {
		for (int i = 0; i < this->check_index.size(); ++i) {
			//cout << this->hint << "[";
			vector<double> single;
			for (auto j : this->check_index.at(i)) {
				single.push_back(this->cal_twoPortal(this->portals.at(i), this->portals.at(j)));
				//cout << single.back()<< " ";
			}
			//cout << "]" << endl;
			this->distance.push_back(single);
		}
	}

	double Portal_Manager::cal_twoPortal(shared_ptr<PortalCrane::Portal>& p0, shared_ptr<PortalCrane::Portal>& p1) {

		double dis = DBL_MAX;
		for (int i = 0; i < 3; ++i) {
			for (int j = 0; j < 3; ++j) {
				// 创建距离请求和结果
				fcl::DistanceRequest<double> request;
				fcl::DistanceResult<double> result;
				// 计算距离
				fcl::distance(p0->collision_objects.at(i), p1->collision_objects.at(j), request, result);
				if (dis > result.min_distance)
					dis = result.min_distance;
			}
		}
		return dis;
	}
	// 连续碰撞检测
	void Portal_Manager::cal_distance(double angle1, double angle2) {
		// 创建碰撞检测请求
		fcl::ContinuousCollisionRequest<double> request;
		
	}

}