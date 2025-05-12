#include <Portal.h>


namespace PortalCrane {
	Portal::Portal() :
		base_position(Eigen::Vector2d{0,0}),
		base_r(8.5),
		connect_t(4.0),
		connect_b(6.5),
		connect_h(25.0),
		end_r(4),
		state(3.1415926/3)
	{
		this->id++;
		this->personal_id = this->id;
		this->is_setparams = true;
		this->connect_bl = this->base_position + Eigen::Vector2d{-connect_b / 2,0 };
		this->connect_br = this->base_position + Eigen::Vector2d{ connect_b / 2,0 };
		this->connect_tr = this->base_position + Eigen::Vector2d{ connect_t / 2,connect_h };
		this->connect_tl = this->base_position + Eigen::Vector2d{-connect_t / 2,connect_h };
		this->end_position = (this->connect_tr + this->connect_tl) / 2;
		this->R_max = this->connect_h + this->end_r;
		this->initialize_state();

	}

	Portal::Portal(Eigen::Vector2d center, double r_base, double t_connect, double b_connect, double h_connect, double r_end, double state):
		base_position(center),
		base_r(r_base),
		connect_t(t_connect),
		connect_b(b_connect),
		connect_h(h_connect),
		end_r(r_end),
		state(state)
	{
		this->id++;
		this->personal_id = this->id;
		this->is_setparams = true;
		this->connect_bl = this->base_position + Eigen::Vector2d{ -connect_b / 2,0 };
		this->connect_br = this->base_position + Eigen::Vector2d{ connect_b / 2,0 };
		this->connect_tr = this->base_position + Eigen::Vector2d{ connect_t / 2,connect_h };
		this->connect_tl = this->base_position + Eigen::Vector2d{ -connect_t / 2,connect_h };
		this->end_position = (this->connect_tr + this->connect_tl) / 2;
		this->R_max = this->connect_h + this->end_r;
		this->initialize_state();
	}

	Portal::Portal(Eigen::Vector2d center, double state) :
		base_position(center),
		base_r(8.5),
		connect_t(4.0),
		connect_b(6.5),
		connect_h(25.0),
		end_r(4),
		state(state)
	{
		this->id++;
		this->personal_id = this->id;
		this->is_setparams = true;
		this->connect_bl = this->base_position + Eigen::Vector2d{ -connect_b / 2,0 };
		this->connect_br = this->base_position + Eigen::Vector2d{ connect_b / 2,0 };
		this->connect_tr = this->base_position + Eigen::Vector2d{ connect_t / 2,connect_h };
		this->connect_tl = this->base_position + Eigen::Vector2d{ -connect_t / 2,connect_h };
		this->end_position = (this->connect_tr + this->connect_tl) / 2;
		this->R_max = this->connect_h + this->end_r;
		this->initialize_state();
	}
#if USE_LIB
	Portal::Portal(struct Gantry_Crane_Info& crane) {
		this->id++;
		this->personal_id = this->id;
		this->is_setparams = true;  

		this->base_position << crane.center.x, crane.center.y;
		this->base_r = crane.rotary_radius;
		this->connect_t = crane.jib_tip_width;
		this->connect_b = crane.jib_rear_width;
		this->connect_h = crane.r;
		this->end_r = sqrt(crane.bucket_size.x * crane.bucket_size.x + crane.bucket_size.y * crane.bucket_size.y) / 2;
		this->state = crane.angle;

		this->connect_bl = this->base_position + Eigen::Vector2d{ -connect_b / 2,0 };
		this->connect_br = this->base_position + Eigen::Vector2d{ connect_b / 2,0 };
		this->connect_tr = this->base_position + Eigen::Vector2d{ connect_t / 2,connect_h };
		this->connect_tl = this->base_position + Eigen::Vector2d{ -connect_t / 2,connect_h };
		this->end_position = (this->connect_tr + this->connect_tl) / 2;
		this->R_max = this->connect_h + this->end_r;
		this->initialize_state();
	}
#endif

	Portal::~Portal() {
		for (auto& obj : this->collision_objects) {
			delete obj;
		}
	}

	vector<pcl::PointXYZ> Portal::get_connect() const {
		vector<pcl::PointXYZ> points;
		points.push_back(pcl::PointXYZ(this->connect_bl_.x(), this->connect_bl_.y(), 0));
		points.push_back(pcl::PointXYZ(this->connect_br_.x(), this->connect_br_.y(), 0));
		points.push_back(pcl::PointXYZ(this->connect_tr_.x(), this->connect_tr_.y(), 0));
		points.push_back(pcl::PointXYZ(this->connect_tl_.x(), this->connect_tl_.y(), 0));
		return points;
	}

	pcl::PointXYZ Portal::get_base() const {
		return pcl::PointXYZ(this->base_position.x(), this->base_position.y(), this->base_r);
	}

	pcl::PointXYZ Portal::get_end() const {
		return pcl::PointXYZ(this->end_position_.x(), this->end_position_.y(), this->end_r);
	}

	void Portal::initialize_state(){
		if (this->is_setparams) {
			this->statetorotate();
			this->refresh_point();
			this->create_fclobj();
			cout << this->hint << this->personal_id << " portal creat success." << endl;
		}
		else {
			cerr << this->hint << this->personal_id << " portal's params is wrong." << endl;
		}
	}

	void Portal::create_fclobj(){
		/* base 圆形对象 */
		std::shared_ptr<fcl::Sphered> sphere_base = make_shared<fcl::Sphered>(this->base_r);
		fcl::Transform3d transform = fcl::Transform3d::Identity();
		transform.translation() = this->augment(this->base_position);
		fcl::CollisionObjectd* obj1 = new fcl::CollisionObjectd(sphere_base, transform);

		this->collision_objects.push_back(obj1);
		  
		/* 梯形凸包对象 */
		std::vector<fcl::Vector3d> vertices;
		vertices.push_back(this->augment(this->connect_bl - this->base_position));
		vertices.push_back(this->augment(this->connect_br - this->base_position));
		vertices.push_back(this->augment(this->connect_tl - this->base_position));
		vertices.push_back(this->augment(this->connect_tr - this->base_position));
		std::vector<int> faces{4,0,1,2,3};
		 
		std::shared_ptr<const std::vector<fcl::Vector3d>> vertices_ptr = std::make_shared<const std::vector<fcl::Vector3d>>(vertices);
		std::shared_ptr<const std::vector<int>> faces_ptr = std::make_shared<const std::vector<int>>(faces);
		shared_ptr<fcl::Convex<double>> convex = make_shared<fcl::Convex<double>>(vertices_ptr, 1, faces_ptr, false);
		fcl::CollisionObjectd* obj2 = new fcl::CollisionObject<double>(convex);
		 
		obj2->setTransform(this->rotate_augment(), this->augment(this->base_position));

		this->collision_objects.push_back(obj2);

		/* 末端圆形 */
		std::shared_ptr<fcl::Sphered> sphere_end = make_shared<fcl::Sphered>(this->end_r);
		transform.translation() = this->augment(this->end_position_);
		fcl::CollisionObjectd* obj3 = new fcl::CollisionObjectd(sphere_end, transform);

		this->collision_objects.push_back(obj3);
	}
	void Portal::refresh_obj() {
		if (this->collision_objects.empty()) {
			cerr << this->hint << this->id << " portal's collision_objects is empty." << endl;
			return;
		}
		this->collision_objects.at(1)->setTransform(this->rotate_augment(), this->augment(this->base_position));
		Eigen::Matrix3d rot;
		rot << 1, 0, 0, 0, 1, 0, 0, 0, 1;
		this->collision_objects.at(2)->setTransform(rot, this->augment(this->end_position_));
	}

	void Portal::set_state(const double state) {
		if (this->collision_objects.empty()) {
			cerr << this->hint << this->id << " portal's collision_objects is empty." << endl;
			return;
		}

		this->state = state;
		this->statetorotate();
		this->refresh_point();
		this->refresh_obj();
	}
	double Portal::get_state() const {
		return this->state;
	}


}