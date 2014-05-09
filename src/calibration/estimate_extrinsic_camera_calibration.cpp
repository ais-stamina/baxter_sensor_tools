/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, University of Bonn, Computer Science Institute VI
 *  Author: Joerg Stueckler, 24.04.2014
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of University of Bonn, Computer Science Institute
 *     VI nor the names of its contributors may be used to endorse or
 *     promote products derived from this software without specific
 *     prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */


#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <rosbag/bag.h>

#include <tf/transform_broadcaster.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <image_geometry/pinhole_camera_model.h>


#include <sensor_msgs/image_encodings.h>

#include <boost/thread.hpp>

#include <tf/transform_listener.h>

#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <std_srvs/Empty.h>


class ChessboardInfo {
public:

	ChessboardInfo() {}

	ChessboardInfo( ros::NodeHandle& nh ) {
		initPose();

    	nh.param( "board_width", size.width, 5 );
    	nh.param( "board_height", size.height, 4 );
    	nh.param( "board_square_size", squareSize, 0.038 );

		ROS_INFO( "using board: %ix%i, square size %f", size.width, size.height, squareSize );

    	initCorners();

	}

	~ChessboardInfo() {}

	void initPose() {
		cv::Mat R = cv::Mat::eye( 3, 3, CV_64FC1 );
		cv::Rodrigues( R, rotation );
		translation = cv::Mat::zeros( 3, 1, CV_64FC1 );
		trackInitialized = false;
	}

	void initCorners() {

		corners.clear();
		for( int i = 0; i < size.height; i++ )
			for( int j = 0; j < size.width; j++ )
				corners.push_back( cv::Point3f( float(j*squareSize), float(i*squareSize), 0.f ) );

	}

	cv::Size size;
	double squareSize;
	std::vector< cv::Point3f > corners;
	cv::Mat rotation, translation;
	bool trackInitialized;

};



// just estimates the pose of a board in the rgb image..
class EstimateCameraExtrinsics
{
public:

	EstimateCameraExtrinsics() {
	}


	~EstimateCameraExtrinsics() {}

    void onInit( ros::NodeHandle& nh ) {

		chessboard_ = ChessboardInfo( nh );

		reset_mean_service_ = nh.advertiseService( "reset_mean", &EstimateCameraExtrinsics::resetMean, this );

		sub_left_inhand_image_ = nh.subscribe( "left_inhand_camera_image", 1, &EstimateCameraExtrinsics::leftInhandImageCallback, this );
		sub_right_inhand_image_ = nh.subscribe( "right_inhand_camera_image", 1, &EstimateCameraExtrinsics::rightInhandImageCallback, this );
		sub_ext_image_ = nh.subscribe( "ext_camera_image", 1, &EstimateCameraExtrinsics::extImageCallback, this );

		sub_left_inhand_camera_info_ = nh.subscribe( "left_inhand_camera_info", 1, &EstimateCameraExtrinsics::leftInhandInfoCallback, this );
		sub_right_inhand_camera_info_ = nh.subscribe( "right_inhand_camera_info", 1, &EstimateCameraExtrinsics::rightInhandInfoCallback, this );
		sub_ext_camera_info_ = nh.subscribe( "ext_camera_info", 1, &EstimateCameraExtrinsics::extInfoCallback, this );

		has_left_inhand_camera_info_ = false;
		has_right_inhand_camera_info_ = false;
		has_ext_camera_info_ = false;

		nh.param<bool>( "debug", debug_, true );

		nh.param< std::string >( "base_frame", base_frame_, "base" );
		nh.param< std::string >( "cam_link_frame", cam_link_frame_, "camera_link" );

		tf_listener_ = boost::shared_ptr< tf::TransformListener >( new tf::TransformListener() );

    }

	bool resetMean( std_srvs::Empty::Request &req, std_srvs::Empty::Response &res ) {

		measurements_.clear();
		return true;

	}



    void update() {

    }


    void leftInhandImageCallback( const sensor_msgs::ImageConstPtr& image ) {

    	if( !has_left_inhand_camera_info_ )
    		return;

		// convert to opencv image
		cv_bridge::CvImagePtr cv_ptr;
		try {
			cv_ptr = cv_bridge::toCvCopy( image, sensor_msgs::image_encodings::BGR8 );
			img_left_inhand_ = cv_ptr->image;
		} catch (const cv_bridge::Exception& error) {
				ROS_ERROR("CvBridge: %s", error.what());
		}

		if( debug_ ) {
			cv::imshow("left_inhand", img_left_inhand_);
			cv::waitKey(1);
		}

    }

    void rightInhandImageCallback( const sensor_msgs::ImageConstPtr& image ) {

    	if( !has_right_inhand_camera_info_ )
    		return;

		// convert to opencv image
		cv_bridge::CvImagePtr cv_ptr;
		try {
			cv_ptr = cv_bridge::toCvCopy( image, sensor_msgs::image_encodings::BGR8 );
			img_right_inhand_ = cv_ptr->image;
		} catch (const cv_bridge::Exception& error) {
				ROS_ERROR("CvBridge: %s", error.what());
		}

		if( debug_ ) {
			cv::imshow("right_inhand", img_right_inhand_);
			cv::waitKey(1);
		}

    }

    void extImageCallback( const sensor_msgs::ImageConstPtr& image ) {

    	if( !has_ext_camera_info_ )
    		return;


		// convert to opencv image
		cv_bridge::CvImagePtr cv_ptr;
		try {
			cv_ptr = cv_bridge::toCvCopy( image, sensor_msgs::image_encodings::BGR8 );
			img_ext_ = cv_ptr->image;
		} catch (const cv_bridge::Exception& error) {
				ROS_ERROR("CvBridge: %s", error.what());
		}

		if( debug_ ) {
			cv::imshow("ext", img_ext_);
			cv::waitKey(1);
		}



    	if( !has_left_inhand_camera_info_ )
    		return;

    	if( img_left_inhand_.empty() )
    		return;

    	if( !has_right_inhand_camera_info_ )
    		return;

    	if( img_right_inhand_.empty() )
    		return;


		// extract checkerboard in inhand and ext images

		// convert to opencv image
		cv::Mat img_ext_gray;
		cv::cvtColor( img_ext_, img_ext_gray, CV_BGR2GRAY );
		Eigen::Matrix4d ext_board_pose = estimateBoardPose( img_ext_gray, ext_camera_info_ );

		cv::Mat img_left_inhand_gray;
		cv::cvtColor( img_left_inhand_, img_left_inhand_gray, CV_BGR2GRAY );
		Eigen::Matrix4d left_inhand_board_pose = estimateBoardPose( img_left_inhand_gray, left_inhand_camera_info_ );

		Eigen::Matrix4d ext_to_left_inhand_cam_pose = left_inhand_board_pose * ext_board_pose.inverse();

		cv::Mat img_right_inhand_gray;
		cv::cvtColor( img_right_inhand_, img_right_inhand_gray, CV_BGR2GRAY );
		Eigen::Matrix4d right_inhand_board_pose = estimateBoardPose( img_right_inhand_gray, right_inhand_camera_info_ );

		Eigen::Matrix4d ext_to_right_inhand_cam_pose = right_inhand_board_pose * ext_board_pose.inverse();

		{

			tf::StampedTransform baseTransform;
			baseTransform.setIdentity();
			try {
				// target, source, time
				// this gets camera to baselink
				tf_listener_->lookupTransform( base_frame_, left_inhand_camera_info_.header.frame_id, left_inhand_camera_info_.header.stamp, baseTransform );
			}
			catch(tf::TransformException& ex) {
				baseTransform.setIdentity();
				ROS_ERROR("Received an exception trying to transform \"%s\" to \"%s\": %s", base_frame_.c_str(), left_inhand_camera_info_.header.frame_id.c_str(), ex.what());
			}

			Eigen::Matrix4d baseTransformEigen = Eigen::Matrix4d::Identity();
			baseTransformEigen.block<3,3>(0,0) = Eigen::Matrix3d( Eigen::Quaterniond( baseTransform.getRotation().w(), baseTransform.getRotation().x(), baseTransform.getRotation().y(), baseTransform.getRotation().z() ) );
			baseTransformEigen(0,3) = baseTransform.getOrigin()[0];
			baseTransformEigen(1,3) = baseTransform.getOrigin()[1];
			baseTransformEigen(2,3) = baseTransform.getOrigin()[2];


			tf::StampedTransform camLinkTransform;
			camLinkTransform.setIdentity();
			try {
				// target, source, time
				// this gets camera to baselink
				tf_listener_->lookupTransform( cam_link_frame_, ext_camera_info_.header.frame_id, ext_camera_info_.header.stamp, camLinkTransform );
			}
			catch(tf::TransformException& ex) {
				baseTransform.setIdentity();
				ROS_ERROR("Received an exception trying to transform \"%s\" to \"%s\": %s", cam_link_frame_.c_str(), ext_camera_info_.header.frame_id.c_str(), ex.what());
			}

			Eigen::Matrix4d camLinkTransformEigen = Eigen::Matrix4d::Identity();
			camLinkTransformEigen.block<3,3>(0,0) = Eigen::Matrix3d( Eigen::Quaterniond( camLinkTransform.getRotation().w(), camLinkTransform.getRotation().x(), camLinkTransform.getRotation().y(), camLinkTransform.getRotation().z() ) );
			camLinkTransformEigen(0,3) = camLinkTransform.getOrigin()[0];
			camLinkTransformEigen(1,3) = camLinkTransform.getOrigin()[1];
			camLinkTransformEigen(2,3) = camLinkTransform.getOrigin()[2];


			Eigen::Matrix4d ext_to_base_pose = baseTransformEigen * ext_to_left_inhand_cam_pose * camLinkTransformEigen.inverse();


			Eigen::Matrix4d T = ext_to_base_pose;
			Eigen::Quaterniond q( T.block<3,3>(0,0) );

			Eigen::Matrix< double, 7, 1 > v;
			v.block<3,1>(0,0) = T.block<3,1>(0,3);
			v(3) = q.x();
			v(4) = q.y();
			v(5) = q.z();
			v(6) = q.w();

			measurements_.push_back( v );

			if( debug_ ) {
				tf::Transform cameraPose;
				cameraPose.setOrigin( tf::Vector3( T(0,3), T(1,3), T(2,3) ) );
				cameraPose.setRotation( tf::Quaternion( q.x(), q.y(), q.z(), q.w() ) );
				tf_broadcaster_.sendTransform( tf::StampedTransform( cameraPose, left_inhand_camera_info_.header.stamp, base_frame_, cam_link_frame_ + "_left_calib" ) );
			}

		}

		{

			tf::StampedTransform baseTransform;
			baseTransform.setIdentity();
			try {
				// target, source, time
				// this gets camera to baselink
				tf_listener_->lookupTransform( base_frame_, right_inhand_camera_info_.header.frame_id, right_inhand_camera_info_.header.stamp, baseTransform );
			}
			catch(tf::TransformException& ex) {
				baseTransform.setIdentity();
				ROS_ERROR("Received an exception trying to transform \"%s\" to \"%s\": %s", base_frame_.c_str(), right_inhand_camera_info_.header.frame_id.c_str(), ex.what());
			}

			Eigen::Matrix4d baseTransformEigen = Eigen::Matrix4d::Identity();
			baseTransformEigen.block<3,3>(0,0) = Eigen::Matrix3d( Eigen::Quaterniond( baseTransform.getRotation().w(), baseTransform.getRotation().x(), baseTransform.getRotation().y(), baseTransform.getRotation().z() ) );
			baseTransformEigen(0,3) = baseTransform.getOrigin()[0];
			baseTransformEigen(1,3) = baseTransform.getOrigin()[1];
			baseTransformEigen(2,3) = baseTransform.getOrigin()[2];


			tf::StampedTransform camLinkTransform;
			camLinkTransform.setIdentity();
			try {
				// target, source, time
				// this gets camera to baselink
				tf_listener_->lookupTransform( cam_link_frame_, ext_camera_info_.header.frame_id, ext_camera_info_.header.stamp, camLinkTransform );
			}
			catch(tf::TransformException& ex) {
				baseTransform.setIdentity();
				ROS_ERROR("Received an exception trying to transform \"%s\" to \"%s\": %s", cam_link_frame_.c_str(), ext_camera_info_.header.frame_id.c_str(), ex.what());
			}

			Eigen::Matrix4d camLinkTransformEigen = Eigen::Matrix4d::Identity();
			camLinkTransformEigen.block<3,3>(0,0) = Eigen::Matrix3d( Eigen::Quaterniond( camLinkTransform.getRotation().w(), camLinkTransform.getRotation().x(), camLinkTransform.getRotation().y(), camLinkTransform.getRotation().z() ) );
			camLinkTransformEigen(0,3) = camLinkTransform.getOrigin()[0];
			camLinkTransformEigen(1,3) = camLinkTransform.getOrigin()[1];
			camLinkTransformEigen(2,3) = camLinkTransform.getOrigin()[2];


			Eigen::Matrix4d ext_to_base_pose = baseTransformEigen * ext_to_right_inhand_cam_pose * camLinkTransformEigen.inverse();


			Eigen::Matrix4d T = ext_to_base_pose;
			Eigen::Quaterniond q( T.block<3,3>(0,0) );

			Eigen::Matrix< double, 7, 1 > v;
			v.block<3,1>(0,0) = T.block<3,1>(0,3);
			v(3) = q.x();
			v(4) = q.y();
			v(5) = q.z();
			v(6) = q.w();

			measurements_.push_back( v );

			if( debug_ ) {
				tf::Transform cameraPose;
				cameraPose.setOrigin( tf::Vector3( T(0,3), T(1,3), T(2,3) ) );
				cameraPose.setRotation( tf::Quaternion( q.x(), q.y(), q.z(), q.w() ) );
				tf_broadcaster_.sendTransform( tf::StampedTransform( cameraPose, right_inhand_camera_info_.header.stamp, base_frame_, cam_link_frame_ + "_right_calib" ) );
			}


		}

		Eigen::Matrix< double, 7, 1 > mean = Eigen::Matrix< double, 7, 1 >::Zero();
		for( unsigned int i = 0; i < measurements_.size(); i++ ) {
			mean += measurements_[i];
		}
		mean /= (double) measurements_.size();

		if( debug_ ) {
			tf::Transform cameraPose;
			cameraPose.setOrigin( tf::Vector3( mean(0), mean(1), mean(2) ) );
			Eigen::Vector4d qm( mean(3), mean(4), mean(5), mean(6) );
			qm.normalize();
			cameraPose.setRotation( tf::Quaternion( qm(0), qm(1), qm(2), qm(3) ) );
			tf_broadcaster_.sendTransform( tf::StampedTransform( cameraPose, ext_camera_info_.header.stamp, base_frame_, cam_link_frame_ + "_mean_calib" ) );
		}

		ROS_INFO_STREAM( "mean pose: " << mean.transpose() );


    }


    Eigen::Matrix4d estimateBoardPose( cv::Mat& img, const sensor_msgs::CameraInfo& camInfo ) {

    	cv::Mat img_viz;
    	if( debug_ )
    		img_viz = img.clone();

		image_geometry::PinholeCameraModel cameraModel;
		cameraModel.fromCameraInfo( camInfo );

		std::vector< cv::Point2f > foundBoardCorners;
		bool boardFound = findChessboardCorners( img, chessboard_.size, foundBoardCorners, cv::CALIB_CB_ADAPTIVE_THRESH );
		if( boardFound ) {

			cv::cornerSubPix( img, foundBoardCorners, cv::Size( 5, 5 ), cv::Size( -1, -1 ), cv::TermCriteria( CV_TERMCRIT_ITER, 20, 1e-2 ) );

			if( debug_ ) {
				// draw found corners..
				cv::drawChessboardCorners( img_viz, chessboard_.size, cv::Mat( foundBoardCorners ), true );

				cv::imshow( camInfo.header.frame_id, img_viz );
			}

			// extract chessboard pose
			// use last pose as initial guess (=> currently not used, only retrieves identity transform from chessboard)
			cv::Mat last_rotation( 3, 1, CV_64FC1 );
			chessboard_.rotation.copyTo( last_rotation );
			cv::Mat last_translation( 3, 1, CV_64FC1 );
			chessboard_.translation.copyTo( last_translation );

			cv::solvePnP( cv::Mat( chessboard_.corners ), cv::Mat( foundBoardCorners ), cameraModel.intrinsicMatrix(), cameraModel.distortionCoeffs(), chessboard_.rotation, chessboard_.translation, false );

			cv::Mat R( 3, 3, CV_64FC1 );
			cv::Rodrigues( chessboard_.rotation, R );
			Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
			for( int y = 0; y < 3; y++ ) {
				for( int x = 0; x < 3; x++ )
					T(y,x) = R.at<double>(y,x);
				T(y,3) = chessboard_.translation.at<double>(y,0);
			}

			if( debug_ ) {
				// visualize by tf
				Eigen::Quaterniond q( T.block<3,3>(0,0) );
				tf::Transform cameraPose;
				cameraPose.setOrigin( tf::Vector3( T(0,3), T(1,3), T(2,3) ) );
				cameraPose.setRotation( tf::Quaternion( q.x(), q.y(), q.z(), q.w() ) );
				tf_broadcaster_.sendTransform( tf::StampedTransform( cameraPose, camInfo.header.stamp, camInfo.header.frame_id, camInfo.header.frame_id + "_board" ) );
			}

			return T;

		}

		return Eigen::Matrix4d::Identity();

    }



    void leftInhandInfoCallback( const sensor_msgs::CameraInfoConstPtr& info ) {

    	has_left_inhand_camera_info_ = true;
    	left_inhand_camera_info_ = *info;

    }

    void rightInhandInfoCallback( const sensor_msgs::CameraInfoConstPtr& info ) {

    	has_right_inhand_camera_info_ = true;
    	right_inhand_camera_info_ = *info;

    }


    void extInfoCallback( const sensor_msgs::CameraInfoConstPtr& info ) {

    	has_ext_camera_info_ = true;
    	ext_camera_info_ = *info;

    }



public:

	sensor_msgs::ImagePtr image_rgb_;

    ChessboardInfo chessboard_;

    ros::Subscriber sub_left_inhand_image_, sub_right_inhand_image_, sub_ext_image_;
    ros::Subscriber sub_left_inhand_camera_info_, sub_right_inhand_camera_info_, sub_ext_camera_info_;

    ros::ServiceServer reset_mean_service_;

    tf::TransformBroadcaster tf_broadcaster_;

    boost::shared_ptr< tf::TransformListener > tf_listener_;

    sensor_msgs::CameraInfo left_inhand_camera_info_, right_inhand_camera_info_, ext_camera_info_;

    bool has_left_inhand_camera_info_, has_right_inhand_camera_info_, has_ext_camera_info_;

    cv::Mat img_left_inhand_, img_right_inhand_, img_ext_;

    std::string base_frame_, cam_link_frame_;

    std::vector< Eigen::Matrix< double, 7, 1 >, Eigen::aligned_allocator< Eigen::Matrix< double, 7, 1 > > > measurements_;

    bool debug_;
};


int main(int argc, char** argv) {

	ros::init(argc, argv, "~");
	ros::NodeHandle n("~");
	EstimateCameraExtrinsics ebp;
	ebp.onInit( n );

	ROS_ERROR("started");


	ros::Rate loop_rate(100);
	while (n.ok())
	{
		ros::spinOnce();
		ebp.update();
		loop_rate.sleep();
	}


}

