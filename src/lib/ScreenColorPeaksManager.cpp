#include "smart_home_scc/ScreenColorPeaksManager.hpp"

#include <sensor_msgs/image_encodings.h>
#include <smart_home_common_msgs/Point.h>
#include <opencv2/photo.hpp>
#include <opencv2/calib3d.hpp>

namespace smart_home {

void get_kmeans_of(cv::Mat& img, int k, int a, double& c, cv::Mat& kmeans_labels, cv::Mat& kmeans_centers)
{
	cv::Mat img_float(img.rows*img.cols, 3, CV_32F);
	for (int j = 0; j < img.rows; j++)
		for (int i = 0; i < img.cols; i++)
			for (int h = 0; h < 3; h++)
				img_float.at<float>(j+(i*img.rows), h) = img.at<cv::Vec3b>(j,i)[h];
	c = cv::kmeans(
		img_float,
		k,
		kmeans_labels,
		cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 10, 1.0),
		a,
		cv::KMEANS_PP_CENTERS,
		kmeans_centers
	);
}

double get_color_score(float r, float g, float b, float c)
{
	cv::Mat3f hsv, rgb(cv::Vec3f(r, g, b));
	cv::cvtColor(rgb, hsv, CV_RGB2HSV);
	cv::Vec3b hsv_px = hsv.at<cv::Vec3b>(0,0);
	double s = hsv_px.val[1], v = hsv_px.val[2];
	return (1-c) * ((0.5*s)/255.0) * ((10*v)/255.0);
}

void set_best_peak(cv::Mat& labels, cv::Mat& centers, unsigned char& r, unsigned char& g, unsigned char& b)
{
	const size_t num_labels = labels.rows;
	std::vector<unsigned int> cluster_counts(centers.rows, 0);
	for (int j = 0; j < num_labels; j++)
		cluster_counts[labels.at<cv::Vec3b>(j,0)[0]]++;

	double max_score = 0;
	for (int i = 0; i < centers.rows; i++)
	{
		float rf = centers.at<float>(i,0);
		float gf = centers.at<float>(i,1);
		float bf = centers.at<float>(i,2);
		double score = get_color_score(rf, gf, bf, static_cast<double>(cluster_counts[i])/num_labels);
		if (score > max_score)
		{
			max_score = score;
			r = static_cast<unsigned char>(rf);
			g = static_cast<unsigned char>(gf);
			b = static_cast<unsigned char>(bf);
		}
	}
}

const std::vector<cv::Point2f> ScreenManager::WORLD_PLANE = {
	cv::Point2f(0,0),
	cv::Point2f(WORLD_WIDTH,0),
	cv::Point2f(0,WORLD_HEIGHT),
	cv::Point2f(WORLD_WIDTH,WORLD_HEIGHT)
};
const cv::Size ScreenManager::WORLD_SIZE = cv::Size(WORLD_WIDTH, WORLD_HEIGHT);

void ColorPeakWindow::push_and_eval(unsigned char r, unsigned char g, unsigned char b, smart_home_common_msgs::ColorPtr& color)
{
	total_r += static_cast<unsigned int>(r);
	total_g += static_cast<unsigned int>(g);
	total_b += static_cast<unsigned int>(b);
	queue_r.push(r);
	queue_g.push(g);
	queue_b.push(b);
	size++;

	while (size > MAX_ELEMS)
	{
		total_r -= static_cast<unsigned int>(queue_r.front());
		total_g -= static_cast<unsigned int>(queue_g.front());
		total_b -= static_cast<unsigned int>(queue_b.front());
		queue_r.pop();
		queue_g.pop();
		queue_b.pop();
		size--;
	}

	color->channels[0] = total_r / size;
	color->channels[1] = total_g / size;
	color->channels[2] = total_b / size;
}

ScreenManager::ScreenManager(
	std::string screen_calibration_request_srv_topic,
	std::string screen_calibration_set_homography_points_srv_topic,
	std::string cap_image_raw_sub_topic,
	std::string color_peak_left_pub_topic,
	std::string color_peak_right_pub_topic,
	std::string color_peaks_telem_pub_topic) :
		color_peak_left_msg(new smart_home_common_msgs::Color),
		color_peak_right_msg(new smart_home_common_msgs::Color),
		color_peaks_telem_msg(new smart_home_scc_msgs::ColorPeaksTelem),
		last_received_frame(nullptr),
		homog_set(false),
		homog(),
		window_left(),
		window_right()
{
	screen_calibration_request_srv = nh.advertiseService(
		screen_calibration_request_srv_topic,
		&smart_home::ScreenManager::screen_calibration_request_callback,
		this
	);
	screen_calibration_set_homography_points_srv = nh.advertiseService(
		screen_calibration_set_homography_points_srv_topic,
		&smart_home::ScreenManager::screen_calibration_set_homography_points_callback,
		this
	);
	cap_image_raw_sub = nh.subscribe(
		cap_image_raw_sub_topic,
		1,
		&smart_home::ScreenManager::cap_image_raw_callback,
		this
	);
	color_peak_left_pub = nh.advertise<smart_home_common_msgs::Color>(
		color_peak_left_pub_topic,
		1
	);
	color_peak_right_pub = nh.advertise<smart_home_common_msgs::Color>(
		color_peak_right_pub_topic,
		1
	);
	color_peaks_telem_pub = nh.advertise<smart_home_scc_msgs::ColorPeaksTelem>(
		color_peaks_telem_pub_topic,
		1
	);

	color_peak_left_msg->channels.resize(3, 0);
	color_peak_right_msg->channels.resize(3, 0);
	color_peaks_telem_msg->intermediate_images.resize(NUM_COLOR_PEAKS_TELEM_IMGS, sensor_msgs::Image());

	ROS_INFO("Initialized ScreenManager.");
}

bool ScreenManager::screen_calibration_request_callback(
	smart_home_scc_msgs::RequestScreenCalibration::Request& req,
	smart_home_scc_msgs::RequestScreenCalibration::Response& res)
{
	bool rc = false;
	cv_bridge::CvImage cv_img_denoised, cv_img_gray, cv_img_blur, cv_img_featin, cv_img_marked;
	std::vector<cv::Point2f> corners;

	const int max_corners = req.max_corners > 0 ? req.max_corners : 35;
	const float quality_level = req.quality_level > 0 ? req.quality_level : 0.01;
	const float min_dist = req.min_dist > 0 ? req.min_dist : 30;
	const bool do_blur = req.do_blur;
	const int kh = req.kh > 0 ? req.kh : 3;
	const int kw = req.kw > 0 ? req.kw : 3;

	if (last_received_frame == nullptr)
	{
		goto END;
	}

	cv::cvtColor(last_received_frame->image, cv_img_gray.image, cv::COLOR_BGR2GRAY);
	cv_img_gray.header = last_received_frame->header;
	cv_img_gray.encoding = sensor_msgs::image_encodings::MONO8;
	res.intermediate_images.push_back(*(cv_img_gray.toImageMsg()));

	if (do_blur)
	{
		cv::blur(cv_img_gray.image, cv_img_blur.image, cv::Size(kw,kh));
		cv_img_blur.header = cv_img_gray.header;
		cv_img_blur.encoding = cv_img_gray.encoding;
		res.intermediate_images.push_back(*(cv_img_blur.toImageMsg()));
		cv_img_featin = cv_img_blur;
	}
	else
	{
		cv_img_featin = cv_img_gray;
	}

	cv::goodFeaturesToTrack(cv_img_featin.image, corners, max_corners, quality_level, min_dist);

	cv_img_marked.image = last_received_frame->image.clone();
	cv_img_marked.header = last_received_frame->header;
	cv_img_marked.encoding = last_received_frame->encoding;
	for (std::vector<cv::Point2f>::iterator iter = corners.begin(); iter != corners.end(); ++iter)
	{
		int x = static_cast<int>(iter->x), y = static_cast<int>(iter->y);
		cv::circle(cv_img_marked.image, cv::Point(x,y), 5, CV_RGB(0,255,0));
		smart_home_common_msgs::Point pt;
		pt.x = x;
		pt.y = y;
		res.corner_positions.push_back(pt);
	}
	res.marked_image = *(cv_img_marked.toImageMsg());
	rc = true;
END:
	res.corners_found = rc;
	return true;
}

bool ScreenManager::screen_calibration_set_homography_points_callback(
	smart_home_scc_msgs::SetScreenCalibrationPointsOfHomography::Request& req,
	smart_home_scc_msgs::SetScreenCalibrationPointsOfHomography::Response& res)
{
	std::vector<cv::Point2f> pts_image(4);
	for (unsigned int i = 0; i < req.pts_of_homog.size(); ++i)
	{
		pts_image[i] = cv::Point2f(req.pts_of_homog[i].x, req.pts_of_homog[i].y);
	}
	homog = cv::findHomography(pts_image, WORLD_PLANE);
	homog_set = true;
	res.successful = true;
}

void ScreenManager::cap_image_raw_callback(const sensor_msgs::ImageConstPtr& msg)
{
	last_received_frame = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	if (homog_set)
	{
		color_peaks_telem_msg->intermediate_images[0] = *(last_received_frame->toImageMsg());

		cv_bridge::CvImage cv_img_world;
		cv::warpPerspective(last_received_frame->image, cv_img_world.image, homog, WORLD_SIZE);
		cv_img_world.header = last_received_frame->header;
		cv_img_world.encoding = last_received_frame->encoding;
		color_peaks_telem_msg->intermediate_images[1] = *(cv_img_world.toImageMsg());

		cv_bridge::CvImage cv_img_rgb;
		cv::cvtColor(cv_img_world.image, cv_img_rgb.image, cv::COLOR_BGR2RGB);
		cv_img_rgb.header = cv_img_world.header;
		cv_img_rgb.encoding = sensor_msgs::image_encodings::RGB8;
		color_peaks_telem_msg->world_frame_image = *(cv_img_rgb.toImageMsg());

		const int k = 4, attempts = 4;
		double compactness_left, compactness_right;
		cv::Mat kmeans_labels_left, kmeans_labels_right, kmeans_centers_left, kmeans_centers_right;
		cv::Range range_height(0, cv_img_rgb.image.rows);
		int width = cv_img_rgb.image.cols;
		int half_width = width / 2;
		cv::Mat cv_img_world_left = cv_img_rgb.image(range_height, cv::Range(0, half_width));
		cv::Mat cv_img_world_right = cv_img_rgb.image(range_height, cv::Range(half_width, width));
		get_kmeans_of(cv_img_world_left, k, attempts, compactness_left, kmeans_labels_left, kmeans_centers_left);
		get_kmeans_of(cv_img_world_right, k, attempts, compactness_right, kmeans_labels_right, kmeans_centers_right);

		unsigned char lr, lg, lb, rr, rg, rb;
		set_best_peak(kmeans_labels_left, kmeans_centers_left, lr, lg, lb);
		set_best_peak(kmeans_labels_right, kmeans_centers_right, rr, rg, rb);

		window_left.push_and_eval(lr, lg, lb, color_peak_left_msg);
		window_right.push_and_eval(rr, rg, rb, color_peak_right_msg);

		color_peak_left_pub.publish(color_peak_left_msg);
		color_peak_right_pub.publish(color_peak_right_msg);
		color_peaks_telem_pub.publish(color_peaks_telem_msg);
	}
}

}
