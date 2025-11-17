#include <algo_search.h>

algo_search::algo_search() {}

void algo_search::init(ros::NodeHandle& nh_)

{

    nh = nh_;

    nh.param<int>("drone_id", drone_id, 0);

    nh.param<std::string>("prefix", prefix, "/drone_");

    nh.param<double>("per_distance", per_distance, 5.0);

    nh.param<double>("Vmax", Vmax, 4.0);

    nh.param<int>("AlgoID/SEARCH_GOAL", algoID, 16);

    local_pos_pub = nh.advertise<msg_set::UsmPositionTargetToStateMachine>(prefix + std::to_string(drone_id) + "/position_cmd/algorithms",
                                                                           10);  // 给usm

    search_area_pub = nh.advertise<std_msgs::Bool>(prefix + std::to_string(drone_id) + "/SearchArea", 100);  // 给调度

    odom_sub = nh.subscribe(prefix + std::to_string(drone_id) + "/mavros/local_position/odom", 10, &algo_search::OdomCallBack, this);  // mavros发布

    home_position_gps_sub_ =
        nh.subscribe(prefix + std::to_string(drone_id) + "/mavros/home_position/home", 10, &algo_search::HomeGpsCallback, this);  // 起飞GPS点

    state_robo_sub = nh.subscribe(prefix + std::to_string(drone_id) + "/state/robotMajorState", 1, &algo_search::m_state_callback, this);  // 状态机发布

    path_points_sub = nh.subscribe(prefix + std::to_string(drone_id) + "/waypoints", 1, &algo_search::path_points_callback, this);

    mission_sub = nh.subscribe(prefix + std::to_string(drone_id) + "/mqtt/group/drone_mission/raw", 1, &algo_search::missionCallBack, this);

    time_per_point = 0.02;

    pid = std::make_shared<PIDController>(1.25, 0.0075, 0.13, Vmax, 2.0d * Vmax);
}

void algo_search::m_state_callback(const std_msgs::Int32ConstPtr& msg)

{

    m_robo_ctrl.data = msg->data;
}

void algo_search::OdomCallBack(const nav_msgs::Odometry::ConstPtr& msg)

{

    drone_odom = *msg;
}

void algo_search::HomeGpsCallback(const mavros_msgs::HomePosition::ConstPtr& msg)

{

    homeposition_gps_ref_ = *msg;
}

/******************************************** 计算航迹
 * ****************************************************/

void algo_search::path_points_callback(const mavros_msgs::WaypointList& msg)
{

    way_point_list_.clear();

    for (const auto& a : msg.waypoints)

    {

        way_point_list_.emplace_back(gps2xyz(a.x_lat, a.y_long, a.z_alt));
    }

    std::reverse(way_point_list_.begin(), way_point_list_.end());

    pub_search_area();

    ROS_INFO("Get Path.");
}

void algo_search::missionCallBack(const msg_set::AssignGroup::ConstPtr& msg)

{

    mission_get = *msg;

    computePtah();

    pub_search_area();

    ROS_INFO("Get Missions.");
}

Point algo_search::gps2xyz(const float& latitude, const float& longitude, const float& altitude)

{

    // 使用PX4源码进行坐标转换:GPS->NED->ENU，将GPS数据角度单位由度--->为弧度

    float lat_rad = latitude * PAI_ / 180.0;

    float lon_rad = longitude * PAI_ / 180.0;

    float ref_lon_rad = homeposition_gps_ref_.geo.longitude * PAI_ / 180.0;

    float ref_lat_rad = homeposition_gps_ref_.geo.latitude * PAI_ / 180.0;

    float sin_lat = sin(lat_rad);  // 序中三角运算使用的是弧度

    float cos_lat = cos(lat_rad);

    float ref_sin_lat = sin(ref_lat_rad);

    float ref_cos_lat = cos(ref_lat_rad);

    float cos_d_lon = cos(lon_rad - ref_lon_rad);

    float arg = ref_sin_lat * sin_lat + ref_cos_lat * cos_lat * cos_d_lon;

    if (arg > 1.0)

        arg = 1.0;

    else if (arg < -1.0)

        arg = -1.0;  // 限幅

    float c = acos(arg);

    float k = (fabs(c) > 0) ? (c / sin(c)) : 1.0;  // c为正数

    // GPS->NED->ENU(x,y对调实现NED->ENU的转换)

    float x = k * cos_lat * sin(lon_rad - ref_lon_rad) * 6371000;  // 地球半径6371000

    float y = k * (ref_cos_lat * sin_lat - ref_sin_lat * cos_lat * cos_d_lon) * 6371000;

    float z = altitude;

    return Point(x, y, z);
}

double algo_search::distance(const Point& start, const Point& end)
{

    return sqrt(pow(end.x - start.x, 2) + pow(end.y - start.y, 2) + pow(end.z - start.z, 2));
}

Point algo_search::interpolate(const Point& start, const Point& end, double t)
{

    Point mid;

    mid.x = start.x + (end.x - start.x) * t;

    mid.y = start.y + (end.y - start.y) * t;

    mid.z = start.z + (end.z - start.z) * t;

    return mid;
}

void algo_search::pub_search_area()

{

    std_msgs::Bool get_search_area;

    get_search_area.data = true;

    search_area_pub.publish(get_search_area);
}

void algo_search::computePtah()

{

    getBoundary();

    generatePoints();

    filterPoints();

    keepInflectionPoint();

    std::reverse(way_point_list_.begin(), way_point_list_.end());
}

void algo_search::getBoundary()
{

    point_list.clear();

    for (auto& a : mission_get.missions[0].area_data.waypoints)

    {

        point_list.emplace_back(gps2xyz(a.x_lat, a.y_long, mission_get.missions[0].height));
    }

    xMin = point_list[0].x;

    xMax = point_list[0].x;

    yMin = point_list[0].y;

    yMax = point_list[0].y;

    if (mission_get.missions[0].area_type == 0)  // circle

    {

        xMin = point_list[0].x - mission_get.missions[0].area_data.waypoints[0].param1;

        xMax = point_list[0].x + mission_get.missions[0].area_data.waypoints[0].param1;

        yMin = point_list[0].y - mission_get.missions[0].area_data.waypoints[0].param1;

        yMax = point_list[0].y + mission_get.missions[0].area_data.waypoints[0].param1;
    }
    else
    {

        for (auto& a : point_list)

        {

            if (a.x > xMax)

            {

                xMax = a.x;
            }

            else if (xMin > a.x)

            {

                xMin = a.x;
            }

            if (a.y > yMax)

            {

                yMax = a.y;
            }

            else if (yMin > a.y)

            {

                yMin = a.y;
            }
        }
    }
}

void algo_search::generatePoints()
{

    all_point.clear();

    int x_count = (xMax - xMin) / per_distance + 1;

    int y_count = (yMax - yMin) / per_distance + 1;

    all_point.emplace_back(xMin, yMin, mission_get.missions[0].height);

    double y_start = yMin;

    for (int j = 1; j <= y_count; ++j)
    {

        y_start += per_distance;

        all_point.emplace_back(xMin, y_start, mission_get.missions[0].height);
    }

    double x_start = xMin;

    for (int i = 1; i <= x_count; ++i)

    {

        x_start += per_distance;

        all_point.emplace_back(x_start, y_start, mission_get.missions[0].height);

        for (int j = 1; j <= y_count; ++j)
        {

            if ((i % 2) == 1)

            {

                y_start -= per_distance;
            }

            else
            {

                y_start += per_distance;
            }

            all_point.emplace_back(x_start, y_start, mission_get.missions[0].height);
        }
    }
}

void algo_search::filterPoints()

{

    way_point_list_.clear();

    if (mission_get.missions[0].area_type == 0)  // circle

    {

        for (auto& a : all_point)

        {

            if (isInsideCircle(a, point_list[0], mission_get.missions[0].area_data.waypoints[0].param1))

            {

                way_point_list_.emplace_back(a);
            }
        }
    }

    else  // polygon

    {

        for (auto& a : all_point)

        {

            if (isInsidePolygon(a, point_list))

            {

                way_point_list_.emplace_back(a);
            }
        }
    }
}

bool algo_search::isInsideCircle(const Point& a, const Point& b, double r)

{

    if ((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) > r * r)

    {

        return false;
    }

    return true;
}

bool algo_search::isInsidePolygon(const Point& point, const std::vector<Point>& polygon)

{

    int count = 0;

    int n = polygon.size();

    for (int i = 0, j = n - 1; i < n; j = i++)

    {

        if (((polygon[i].y > point.y) != (polygon[j].y > point.y))

            && (point.x < (polygon[j].x - polygon[i].x) * (point.y - polygon[i].y) / (polygon[j].y - polygon[i].y) + polygon[i].x))

        {

            count++;
        }
    }

    return (count % 2) == 1;
}

void algo_search::keepInflectionPoint()

{

    double prex, prey, nexx, nexy;

    std::vector<Point> tem_points;

    tem_points.emplace_back(way_point_list_[0]);

    for (int i = 1; i < way_point_list_.size() - 1; ++i)

    {

        prex = way_point_list_[i].x - way_point_list_[i - 1].x;

        prey = way_point_list_[i].y - way_point_list_[i - 1].y;

        nexx = way_point_list_[i + 1].x - way_point_list_[i].x;

        nexy = way_point_list_[i + 1].y - way_point_list_[i].y;

        if (abs(prex) + abs(nexy) > 9.999999999 || abs(prey) + abs(nexx) > 9.99999999)

        {

            tem_points.emplace_back(way_point_list_[i]);
        }
    }

    tem_points.emplace_back(way_point_list_.back());

    way_point_list_ = std::move(tem_points);
}

/******************************************** 执行航迹飞行
 * ****************************************************/

void algo_search::search_zone()

{

    if (way_point_list_.size() < 1)

    {

        ROS_WARN_THROTTLE(5, "no path get OR arrived~~~");

        processArrived();

        return;
    }

    processflight();

    if (hasReachedWaypoint(way_point_list_.back()))

    {

        ROS_INFO("%d points left", way_point_list_.size());

        way_point_list_.pop_back();
    }
}

void algo_search::processArrived()
{

    target_ctl.header.frame_id = "map";

    target_ctl.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

    target_ctl.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ |

                           mavros_msgs::PositionTarget::IGNORE_VX | mavros_msgs::PositionTarget::IGNORE_VY | mavros_msgs::PositionTarget::IGNORE_VZ |

                           mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    target_ctl.position.x = drone_odom.pose.pose.position.x;

    target_ctl.position.y = drone_odom.pose.pose.position.y;

    target_ctl.position.z = drone_odom.pose.pose.position.z;

    target_ctl.yaw = yaw_ctl;

    machine_target.pos_target = target_ctl;

    machine_target.algo_id = algoID;

    machine_target.algo_hold_state = true;

    local_pos_pub.publish(machine_target);
}

void algo_search::processflight()
{

    double dist = distance(Point(drone_odom.pose.pose.position.x, drone_odom.pose.pose.position.y, drone_odom.pose.pose.position.z), way_point_list_.back());

    double cur_spd = distance(Point(drone_odom.twist.twist.linear.x, drone_odom.twist.twist.linear.y, drone_odom.twist.twist.linear.z), Point(0, 0, 0));

    double velocity_adjustment = pid->update(dist, cur_spd, 0.1d);

    double spd_ctl = velocity_adjustment + cur_spd;

    target_ctl.header.frame_id = "map";

    target_ctl.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

    target_ctl.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ |

                           mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY | mavros_msgs::PositionTarget::IGNORE_PZ |

                           mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    double delta_y = way_point_list_.back().y - drone_odom.pose.pose.position.y;

    double delta_x = way_point_list_.back().x - drone_odom.pose.pose.position.x;

    double delta_height = way_point_list_.back().z - drone_odom.pose.pose.position.z;

    if (std::abs(delta_y) > 0.5d | std::abs(delta_x) > 0.5d)

    {

        yaw_ctl = std::atan2(delta_y, delta_x);
    }

    if (std::abs(delta_height) > 0.5d)

    {

        pitch_ctl = std::atan2(delta_height, dist);
    }

    target_ctl.velocity.x = spd_ctl * std::cos(yaw_ctl);

    target_ctl.velocity.y = spd_ctl * std::sin(yaw_ctl);

    target_ctl.velocity.z = spd_ctl * std::sin(pitch_ctl);

    target_ctl.yaw = yaw_ctl;

    machine_target.pos_target = target_ctl;

    machine_target.algo_id = algoID;

    machine_target.algo_hold_state = false;

    local_pos_pub.publish(machine_target);
}

bool algo_search::hasReachedWaypoint(const Point& waypoint)

{

    double dx = drone_odom.pose.pose.position.x - waypoint.x;

    double dy = drone_odom.pose.pose.position.y - waypoint.y;

    double dz = drone_odom.pose.pose.position.z - waypoint.z;

    double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    return distance < 3.d;
}

void algo_search::if_search()

{

    if (m_robo_ctrl.data == algoID)

        search_zone();
}