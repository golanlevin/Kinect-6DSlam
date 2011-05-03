/*
 *  Slam.cpp
 *  ofxKinect
 *
 *  Created by Minjae Lee on 4/17/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "Slam.h"

// This might change since this is found in trial-error technique
#define INDEX(x, y) (((w) - (x)) + (y) * (w))

inline std::string Slam::to_string(const int t, int width)
{
	std::stringstream ss;
	ss << std::setfill('0') << std::setw(width) << t;
	
	return ss.str();
}

void Slam::init()
{
	slam_mode = INACTIVE;
	scan_count = 0;
	vps.clear();
}

void Slam::toggle()
{
	if(slam_mode == INACTIVE)
	{
		scan_count = 0;
		max_box = 0;
		slam_mode = RECORD;
		
		std::cout << "Recording..." << std::endl;
	}
	else if(slam_mode == RECORD)
	{
		slam_mode = SAVE;
		
		std::cout << "Saving..." << std::endl;
	}
}

void Slam::update(ofxKinect &kinect, ofxCvGrayscaleImage &mask)
{
	if(slam_mode == RECORD)
	{
		gen_record(kinect, mask, 640, 480, 1);
	}
	else if(slam_mode == SAVE)
	{
		gen_save("", 360);
	}
}

void Slam::gen_record(ofxKinect &kinect, ofxCvGrayscaleImage &mask, size_t w, size_t h, size_t step)
{
	Points pts;
	
	unsigned char *mask_pixels = mask.getPixels();
	
	for(size_t y = 0; y < h; y += step)
	{
		for(size_t x = 0; x < w; x += step)
		{
			size_t i = INDEX(x, y);
			
			if(mask_pixels[i] == 0) continue;
			
			ofxVec3f p = kinect.getWorldCoordinateFor(x, y);
			Vector3 v;
			
			v.x = p.x;
			v.y = p.y;
			v.z = p.z;
			
			int maxp = max(abs(v.x), max(abs(v.y), abs(v.z)));
			
			if(maxp > max_box) max_box = maxp;
			
			pts.push_back(v);
		}
	}
	
	vps.push_back(pts);
}

void Slam::gen_save(std::string path, float total_deg)
{
	// Assume that we turned total_deg with approximately same dt
	int j = 0;
	float d = total_deg / float(vps.size());
	
	for(float i = 0.0; i < total_deg; i += d)
	{
		std::string scanname = path + "scan" + to_string(j, 3) + ".3d";
		std::string rotname = path + "scan" + to_string(j, 3) + ".pose";
		
		FILE *fd = fopen(scanname.c_str(), "w+");
		FILE *fe = fopen(rotname.c_str(), "w+");
		
		if(fd && fe)
		{
			fprintf(fd, "%dx%d\n", max_box + 1, max_box + 1);
			
			for(size_t k = 0; k < vps[j].size(); k++)
			{
				float x = vps[j].at(k).x;
				float y = vps[j].at(k).y;
				float z = vps[j].at(k).z;
				
				fprintf(fd, "%f %f %f\n", x, y, z);
			}
			
			fprintf(fe, "0 0 0\n0 %f 0", i);
			
			fclose(fd);
			fclose(fe);
		}
		
		j++;
	}
	
	vps.clear();
	slam_mode = INACTIVE;
}

void Slam::alt_init()
{
	scan_alt_count = 0;
}

void Slam::alt_gen_scan(ofxKinect &kinect, ofxCvGrayscaleImage &mask, std::string path, size_t w, size_t h, size_t step)
{
	std::string scanname = path + "scan" + to_string(scan_alt_count, 3) + ".3d";
	FILE *fd = fopen(scanname.c_str(), "w+");
	
	unsigned char *mask_pixels = mask.getPixels();
	
	if(fd)
	{
		unsigned int maxp = 0;
		
		for(size_t y = 0; y < h; y += step)
		{
			for(size_t x = 0; x < w; x += step)
			{
				size_t i = INDEX(x, y);
				
				if(mask_pixels[i] == 0) continue;
				
				ofxVec3f p = kinect.getWorldCoordinateFor(x, y);
				Vector3 v;
				
				v.x = p.x;
				v.y = p.y;
				v.z = p.z;
				
				int reach = max(abs(v.x), max(abs(v.y), abs(v.z)));
				
				if(reach > maxp) maxp = reach;
			}
		}
		
		fprintf(fd, "%dx%d\n", maxp + 1, maxp + 1);
		
		for(size_t y = 0; y < h; y += step)
		{
			for(size_t x = 0; x < w; x += step)
			{
				size_t i = INDEX(x, y);
				
				if(mask_pixels[i] == 0) continue;
				
				ofxVec3f p = kinect.getWorldCoordinateFor(x, y);
				Vector3 v;
				
				v.x = p.x;
				v.y = p.y;
				v.z = p.z;
				
				fprintf(fd, "%f %f %f\n", v.x, v.y, v.z);
			}
		}
		
		fclose(fd);
		scan_alt_count++;
	}
}

void Slam::alt_gen_pose(std::string path, float total_deg)
{
	int j = 0;
	float d = total_deg / float(scan_alt_count);
	
	for(float i = 0.0; i < total_deg; i += d)
	{
		std::string rotname = path + "scan" + to_string(j, 3) + ".pose";
		
		FILE *fd = fopen(rotname.c_str(), "w+");
		
		if(fd)
		{
			fprintf(fd, "0 0 0\n0 %f 0", i);
			fclose(fd);
		}
		
		j++;
	}
	
	scan_alt_count = 0;
}
