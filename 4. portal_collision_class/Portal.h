#ifndef PORTAL_H_
#define PORTAL_H_

#include <iostream>
#include <Eigen/Dense>
#include <string>
#include <vector>

#include <fcl/geometry/shape/sphere.h>
#include <fcl/geometry/shape/convex.h>
#include <fcl/narrowphase/collision_object.h>


#include <pcl/point_types.h>

#define USE_LIB false

#if USE_LIB
#include <AntiCollisionForGantryCrane.h>
#endif

using namespace std;

namespace PortalCrane {
	class Portal {
	public:
		static int id;
	private:
		string hint = "[Portal_id]:";
		bool is_setparams = false;
		/* 基座信息 */
		double base_r;

		/* 连杆的上底、下底、高 四个点坐标*/ 
		double connect_t, connect_b, connect_h;
		Eigen::Vector2d connect_bl, connect_br,connect_tl, connect_tr;
		Eigen::Vector2d connect_bl_, connect_br_, connect_tl_, connect_tr_;

		/* 末端圆心、半径 */ 
		Eigen::Vector2d end_position;
		Eigen::Vector2d end_position_;
		double end_r;

		// 门机旋转矩阵 与 state关联
		Eigen::Matrix2d rotate;

	public:
		int personal_id;
		/* FCL 对象 */
		vector<fcl::CollisionObjectd*> collision_objects;
		// 可能发生碰撞的距离
		double R_max;
		// 门机状态，即旋转角度(弧度制)
		double state;
		// 基座坐标
		Eigen::Vector2d base_position;
	public:
		Portal();
		Portal(Eigen::Vector2d center, double r_base, double t_connect, double b_connect, double h_connect, double r_end, double state);
		Portal(Eigen::Vector2d center, double state);
#if USE_LIB
		Portal(struct Gantry_Crane_Info& crane);
#endif
		~Portal();

		void set_state(const double state);

		vector<pcl::PointXYZ> get_connect() const;
		pcl::PointXYZ get_base() const;
		pcl::PointXYZ get_end() const;
		double get_state() const;
		inline void refresh_connect(double h);

	private:
		void initialize_state();
		void create_fclobj();
		void refresh_obj();

		inline void statetorotate();
		inline void refresh_point();
		inline fcl::Vector3d augment(const Eigen::Vector2d& vec);
		inline Eigen::Matrix3d rotate_augment();


	};
	inline void Portal::statetorotate() {
		this->rotate << cos(this->state - 1.584757), sin(this->state - 1.584757),
						-sin(this->state - 1.584757), cos(this->state - 1.584757);
	}

	inline void Portal::refresh_point() {
		this->connect_bl_ = this->rotate * (this->connect_bl - this->base_position) + this->base_position;
		this->connect_br_ = this->rotate * (this->connect_br - this->base_position) + this->base_position;
		this->connect_tr_ = this->rotate * (this->connect_tr - this->base_position) + this->base_position;
		this->connect_tl_ = this->rotate * (this->connect_tl - this->base_position) + this->base_position;
		this->end_position_ = this->rotate * (this->end_position - this->base_position) + this->base_position;
	}
	inline fcl::Vector3d Portal::augment(const Eigen::Vector2d& vec) {
		return fcl::Vector3d(vec.x(), vec.y(), 0);
	}
	inline Eigen::Matrix3d Portal::rotate_augment() {
		Eigen::Matrix3d rot;
		rot << this->rotate.row(0), 0,
			this->rotate.row(1), 0,
			0, 0, 1;
		return rot;
	}
	inline void Portal::refresh_connect(double h) {
		this->connect_h = h;

		this->connect_bl = this->base_position + Eigen::Vector2d{ -connect_b / 2,0 };
		this->connect_br = this->base_position + Eigen::Vector2d{ connect_b / 2,0 };
		this->connect_tr = this->base_position + Eigen::Vector2d{ connect_t / 2,connect_h };
		this->connect_tl = this->base_position + Eigen::Vector2d{ -connect_t / 2,connect_h };
		this->end_position = (this->connect_tr + this->connect_tl) / 2;
		this->R_max = this->connect_h + this->end_r;
		//this->initialize_state();
		this->statetorotate();
		this->refresh_point();
		//cout << this->hint << this->personal_id << " portal refesh success." << endl;

		/* 梯形凸包对象 */
		std::vector<fcl::Vector3d> vertices;
		vertices.push_back(this->augment(this->connect_bl - this->base_position));
		vertices.push_back(this->augment(this->connect_br - this->base_position));
		vertices.push_back(this->augment(this->connect_tl - this->base_position));
		vertices.push_back(this->augment(this->connect_tr - this->base_position));
		std::vector<int> faces{ 4,0,1,2,3 };
		 
		std::shared_ptr<const std::vector<fcl::Vector3d>> vertices_ptr = std::make_shared<const std::vector<fcl::Vector3d>>(vertices);
		std::shared_ptr<const std::vector<int>> faces_ptr = std::make_shared<const std::vector<int>>(faces);
		shared_ptr<fcl::Convex<double>> convex = make_shared<fcl::Convex<double>>(vertices_ptr, 1, faces_ptr, false);
		fcl::CollisionObjectd* obj2 = new fcl::CollisionObject<double>(convex);

		obj2->setTransform(this->rotate_augment(), this->augment(this->base_position));

		// 替换掉原来的第二个元素
		if (this->collision_objects.size() > 1) {
			delete this->collision_objects[1];
			this->collision_objects[1] = obj2; 
		}
		else {
			cerr << this->hint << this->id << "Error: collision_objects does not have enough elements." << endl;
		}
	}
					
}

#endif