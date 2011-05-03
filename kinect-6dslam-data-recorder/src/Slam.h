/*
 *  Slam.h
 *  ofxKinect
 *
 *  Convert pcloud into .3d (pcloud) and .pose (rotation) file s.t. Slam6D can parse it.
 *
 *  Created by Minjae Lee on 4/17/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */
#include <vector>
#include <string>
#include "ofMain.h"
#include "ofxKinect.h"
#include "ofxOpenCv.h"

/**
 * Vector in 3D space
 */
class Vector3
{
public:
	float x, y, z;
};

/**
 * Slam class to deal with recording pclouds and outputing them
 *
 * Provides 2 modes, one is continuous mode, which runs in real-time and store all pclouds until user claims
 * recording phase is over, and run into saving phase, which saves all pcloud data and rotation data at once.
 * Another one is static mode, which stores pcloud data whenever user claims to capture (like taking photo).
 * After user claims that all capturing is over, then it saves all rotation data (aka alt_mode)
 *
 * Key assumption of this technique is that for the first mode, it assumes that rotations happen in real-time
 * in uniformly continuous way s.t. capturing all data is done quickly, and uniformly. This is dangerous assumption
 * to have, since it is highly dependent on performance of MAC and lazy susan.
 * Second mode's assumption is that every rotation is uniformly distributed in 360 deg (you can easily modify it,
 * but as default). This mode removes continuous nature away, but it requires more time since object has to be
 * placed meticulously every time user wants to capture one rotation.
 */
class Slam
{
public:
	typedef std::vector<Vector3> Points;
	
	// Mode indicating whether we are saving points for SLAM
	enum RECSLAM_MODE
	{
		INACTIVE, // Nothing is happening
		RECORD, // Recording points
		SAVE // Saving phase to save recorded pclouds so far
	};
	
	// Member variables
	RECSLAM_MODE slam_mode;
	std::vector<Points> vps;
	
	// A "Box" which contains all point data within itself (boundary box)
	int max_box;
	
	// How many scans we performed so far
	size_t scan_count;
	size_t scan_alt_count;
	
	// Member functions
	// Change given int t into string with 0 padding of size given width
	std::string to_string(const int t, int width);

	// First mode
	// Initialize all data
	void init();
	
	// Change mode (INACTIVE -> RECORD, RECORD -> SAVE ... -> INACTIVE)
	void toggle();
	
	// Store pcloud for every update (depend on performance of MAC)
	void update(ofxKinect &kinect, ofxCvGrayscaleImage &mask);
	
	// Store pcloud
	void gen_record(ofxKinect &kinect, ofxCvGrayscaleImage &mask, size_t w, size_t h, size_t step);
	
	// Output to file (pcloud, rotation) to given path & degree
	void gen_save(std::string path, float total_deg);
	
	// Second mode
	// Initialize all data
	void alt_init();
	
	// Output pcloud to file to given path
	void alt_gen_scan(ofxKinect &kinect, ofxCvGrayscaleImage &mask, std::string path, size_t w, size_t h, size_t step);
	
	// Output rotation to file to given path
	void alt_gen_pose(std::string path, float total_deg);
};