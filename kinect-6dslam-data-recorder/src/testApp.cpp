#include "testApp.h"


//--------------------------------------------------------------
void testApp::setup()
{
	kinect.init();
	kinect.setVerbose(true);
	kinect.open();
	kinect.enableDepthNearValueWhite(true);
	
	DHX = new DatumHistorian(16);
	DHY = new DatumHistorian(16);
	DHZ = new DatumHistorian(16);
	
	KW = kinect.width;
	KH = kinect.height;

	colorImg.allocate				(KW,KH);
	ofxCv8uC1_Temp1.allocate		(KW,KH);
	ofxCv8uC1_Temp2.allocate		(KW,KH);
	
	ofxCv8uC1_Depth.allocate		(KW,KH);
	ofxCv8uC1_DepthRaw.allocate		(KW,KH);
	ofxCv8uC1_DepthRawThreshed.allocate	(KW,KH);
	
	ofxCv8uC1_DepthBlobs.allocate	(KW,KH);
	ofxCv8uC1_ThreshN.allocate		(KW,KH);
	ofxCv8uC1_ThreshF.allocate		(KW,KH);
	ofxCv8uC1_ThreshNF.allocate		(KW,KH);
	
	ofxCv8uC1_DepthBg.allocate		(KW,KH);
	ofxCv8uC1_DepthBgDif.allocate	(KW,KH);
	ofxCv8uC1_DepthBgDifThresh.allocate	(KW,KH);
	
	ofxCv8uC1_DepthFrameDif.allocate(KW,KH);
	
	
	cvImgTemp = cvCreateImage( cvSize(KW,KH), IPL_DEPTH_8U, 1); 
	MAX_N_CONTOUR_POINTS = 4000;
	cvpts    = new CvPoint*[1];
	cvpts[0] = new CvPoint[MAX_N_CONTOUR_POINTS];
	for (int i=0; i<MAX_N_CONTOUR_POINTS; i++){
		cvpts[0][i] = cvPoint(0,0);
	}
	ncvpts = new int[1];
	ncvpts[0] = 0;
	
	
	
	
	kinectPrevFrameMillis = 0;
	
	gW = 256;
	gH = (gW*3)/4;
	gM = 8; 

	
	// ofSetFrameRate(60);
	ofSetVerticalSync(false);
	
		
	gui.setup("App Controls", gM, gH*2+gM*3, gW, 300);
	gui.addPanel(" Main Controls", 1, false);
	gui.addPanel(" Foreground Finding", 1, false);
	gui.addPanel(" Slam input file generator", 1, false);
	
	//--------- PANEL 0
	gui.setWhichPanel(0);
	gui.setWhichColumn(0);
	gui.addSlider("Far Threshold",		"LO_THRESHOLD", 30, 0, 255, true);	
	gui.addSlider("Near Threshold",		"HI_THRESHOLD", 255, 0, 255, true);	
	gui.addToggle("DoDepth BgSubtract?","DO_DEPTH_BGSUB", 1);
	gui.addToggle("Capture DepthBg (x)","CAPTURE_DEPTH_BG", 0);
	
	//--------- PANEL 1
	gui.setWhichPanel(1);
	gui.setWhichColumn(0);
	gui.addSlider("BgSub Threshold",	"BG_SUB_THRESHOLD", 3, 0,25, true);
	gui.addSlider("Minimum Fg Size",	"FG_MIN_SIZE",		30, 1, 100, true);	
	
	
	//--------- PANEL 2
	gui.setWhichPanel(2);
	gui.setWhichColumn(0);
	
	gui.loadSettings("controlPanelSettings.xml");
	
	
	slam.init();
	slam.alt_init();
}




//--------------------------------------------------------------
void testApp::update()
{
	ofBackground(100, 100, 100);
	gui.update();
	kinect.update();
	if (kinect.isFrameNew()){
		
		// Compute Kinect frame rate
		computeFrameRate();
		
		// Fetch values from the control panel
		int nearThreshold		= gui.getValueI("LO_THRESHOLD");
		int farThreshold		= gui.getValueI("HI_THRESHOLD");
	
		bool bDoBgSubtraction   = gui.getValueB("DO_DEPTH_BGSUB");
		bool bCurrCaptureBg		= gui.getValueB("CAPTURE_DEPTH_BG");
		bool bDoCaptureTheBg	= ((bCurrCaptureBg != bLastCaptureBg) && bCurrCaptureBg);
		bLastCaptureBg = bCurrCaptureBg;
		
		// Fetch the color image, just for display purposes.
		colorImg.setFromPixels(kinect.getPixels(), KW,KH);
		colorImg.mirror(false, true);
	
		
		// Retrieve the current depth buffer.
		ofxCv8uC1_DepthRaw.setFromPixels(kinect.getDepthPixels(), KW,KH);
		ofxCv8uC1_DepthRaw.mirror(false, true);

		// Compute a double-ended threshold of the depth image
		ofxCv8uC1_ThreshN = ofxCv8uC1_DepthRaw;
		ofxCv8uC1_ThreshF = ofxCv8uC1_DepthRaw;
		ofxCv8uC1_ThreshN.threshold(nearThreshold, false);
		ofxCv8uC1_ThreshF.threshold(farThreshold,  true);
		cvAnd(	ofxCv8uC1_ThreshN.getCvImage(), 
				ofxCv8uC1_ThreshF.getCvImage(), 
				ofxCv8uC1_ThreshNF.getCvImage(), NULL);
		cvAnd(	ofxCv8uC1_ThreshNF.getCvImage(), 
				ofxCv8uC1_DepthRaw.getCvImage(), 
				ofxCv8uC1_DepthRawThreshed.getCvImage(), NULL);


		ofxCv8uC1_Depth = ofxCv8uC1_DepthRawThreshed;
		ofxCv8uC1_Depth.flagImageChanged();
		
		
		//-------------------------------------------------
		if (bDoBgSubtraction){
			// Do background subtraction on the depth image. 
			// Things will run slower, but your hand doesn't have to be frontmost to be tracked.
			
			if (bDoCaptureTheBg){			
				captureBackground(); 
			}
			processBackground();
			computeForegroundBlobs();

			
		} else {
			// Don't do background subtraction on the depth image. 
			// Everything will run a lot faster, but your hand really has to be frontmost. 
			gui.setValueB("CAPTURE_DEPTH_BG", false, 0);
		}
		
		
		slam.update(kinect, ofxCv8uC1_DepthBlobs);

	}
}








//--------------------------------------------------------------
void testApp::captureBackground(){
	bool bDoBgSubtraction = gui.getValueB("DO_DEPTH_BGSUB");
	if (bDoBgSubtraction){
		gui.setValueB("CAPTURE_DEPTH_BG", false, 0);
		ofxCv8uC1_DepthBg = ofxCv8uC1_Depth;
		for (int i=0; i<3; i++){
			ofxCv8uC1_DepthBg.dilate_3x3();
		}
	}
}

//--------------------------------------------------------------
void testApp::processBackground(){
	
	// Background subtraction: compute the difference 
	// between the current background and the stored background, 
	// and put this difference into ofxCv8uC1_DepthBgDif
	cvSub(ofxCv8uC1_Depth.getCvImage(), 
		  ofxCv8uC1_DepthBg.getCvImage(), 
		  ofxCv8uC1_DepthBgDif.getCvImage(), NULL); 
	ofxCv8uC1_DepthBgDifThresh = ofxCv8uC1_DepthBgDif;
	
	// Threshold the difference image, so that white indicates
	// places where there is a significant difference from the background. 
	// Erode this to eliminate twinkly noise, which is characteristic of 
	// the Kinect's depth images at the edges of objects. 
	int bgSubThreshold = gui.getValueI("BG_SUB_THRESHOLD", 0);
	ofxCv8uC1_DepthBgDifThresh.threshold(bgSubThreshold,  false);
	ofxCv8uC1_DepthBgDifThresh.erode_3x3();

}

//--------------------------------------------------------------
void testApp::computeForegroundBlobs(){
	// compute the contours (blobs) of the difference-from-background image. 
	// filter these by size to extract only non-noise blobs (e.g. hands).
	
	// Find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
	// Note: "find holes" is set to false so we just get the exterior contour.
	int minFgSize = gui.getValueI("FG_MIN_SIZE", 0); 
	int minContourArea = minFgSize * minFgSize;
	int maxContourArea = KW*KH;
	contourFinder.findContours(ofxCv8uC1_DepthBgDifThresh, minContourArea, maxContourArea, 10, false);
	
	
	// Into the ofxCv8uC1_DepthBlobs image, render filled versions of the blobs, as a mask
	vector<ofxCvBlob> blobs = contourFinder.blobs;
	CvScalar color_white = CV_RGB(255,255,255);
	CvScalar color_black = CV_RGB(0,0,0);
	int nBlobs = blobs.size();
	int nCvImgTempPix = cvImgTemp->widthStep * cvImgTemp->height;
	for (int i=0; i<nCvImgTempPix; i++){ cvImgTemp->imageData[i] = 0; } // blank cvImgTemp

	// For each blob, fill it with a solid color. This eliminates holes, too. 
	for (int i=0; i<nBlobs; i++){
		ofxCvBlob blobi = blobs[i];
		vector <ofPoint>    pts  = blobi.pts;    // the contour of the blob
		int                 nPts = blobi.nPts;   // number of pts;
		if (nPts > 1){
			nPts = MIN(MAX_N_CONTOUR_POINTS, nPts); 
			for (int j=0; j<nPts; j++){ 
				ofPoint pt = pts[j];
				int px = (int) pt.x; // 0...imgw
				int py = (int) pt.y; // 0...imgh
				cvpts[0][j].x = px;
				cvpts[0][j].y = py;
			}
			ncvpts[0] = nPts;
			cvFillPoly(cvImgTemp, cvpts, ncvpts, 1, color_white, 8, 0 );
		}
		
	}
	ofxCv8uC1_DepthBlobs = cvImgTemp;
	
	// Re-color this difference-from-background image such that
	// white areas are now colored with their true depth color.
	// Voila: a depth-image containing strictly "new" objects.
	cvAnd(ofxCv8uC1_DepthBlobs.getCvImage(), 
		  ofxCv8uC1_Depth.getCvImage(), 
		  ofxCv8uC1_DepthBlobs.getCvImage(), NULL);
	ofxCv8uC1_DepthBlobs.flagImageChanged();	
}




//--------------------------------------------------------------
void testApp::draw()
{
	glColor3f(1,1,1);
	colorImg.draw					(gM*2+gW*1,	gM*1,		gW, gH);
	ofxCv8uC1_Depth.draw			(gM*1+gW*0,	gM*1,		gW, gH);
	
	if (gui.getValueB("DO_DEPTH_BGSUB")){
		ofxCv8uC1_DepthBg.draw		(gM*2+gW*1, gM*2+gH,	gW,	gH);
		ofxCv8uC1_DepthBlobs.draw	(gM*1+gW*0, gM*2+gH,	gW,	gH);
		contourFinder.draw			(gM*1+gW*0, gM*2+gH,	gW,	gH);
		
	}
	
	float sc = 2.0;//2.75;
	float gr = 1.0;
	glColor3f(gr,gr,gr);
	ofxCv8uC1_Depth.draw			(gM*3+gW*2,	gM*1,		gW*sc, gH*sc);
	//ofxCv8uC1_DepthBlobs.draw		(gM*3+gW*2,	gM*1,		gW*sc, gH*sc);


	char reportStr[1024];
	sprintf(reportStr, "fps: %f", kinectFrameRate);//ofGetFrameRate());
	ofSetColor(255, 255, 255);
	ofDrawBitmapString(reportStr, gW*0+gM*1, ofGetHeight()-gM);
	
	gui.draw();
}









void testApp::drawPointCloud() {
	int step = 2;
	ofScale(1. / step, 1. / step, 1. / step);
	// two magic numbers derived visually
	float scaleFactor = .0021;
	float minDistance = -10;
	int w = 640;
	int h = 480;
	ofTranslate(w / 2, h / 2);
	ofRotateY(mouseX);
	ofTranslate(-w / 2, -h / 2);
	float* distancePixels = kinect.getDistancePixels();
	glBegin(GL_POINTS);
	for(int y = 0; y < h; y += step) {
		for(int x = 0; x < w; x += step) {
			int i = y * w + x;
			float z = distancePixels[i];
			float scaledx = (x - w / 2) * (z + minDistance) * scaleFactor;
			float scaledy = (y - h / 2) * (z + minDistance) * scaleFactor;
			glVertex3f(scaledx, scaledy, z);
		}
	}
	glEnd();
}

//--------------------------------------------------------------
void testApp::exit(){
	kinect.close();
}

//--------------------------------------------------------------
void testApp::keyPressed (int key)
{
	switch (key)
	{
		
		case 'x':
		case 'X':
			captureBackground();
			break;
		case 'a':
		case 'A':
			slam.toggle();
			break;
		case 's':
		case 'S':
			slam.alt_gen_scan(kinect, ofxCv8uC1_DepthBlobs, "", 640, 480, 1);
			break;
		case 'd':
		case 'D':
			slam.alt_gen_pose("", 360);
			break;
	}
}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y)
{}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button){
	gui.mouseDragged(x, y, button);
}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button){
	//printf("%d button\n", button); 
	if (button == 2){
		// right click learns background;
		captureBackground();
	} else {
		
		gui.mousePressed(x, y, button);
		
	}
}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button){
	gui.mouseReleased();
}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h){
}


//--------------------------------------------------------------
void testApp::computeFrameRate(){
	float now = ofGetElapsedTimeMillis();
	float FR = 1000.0/(now - kinectPrevFrameMillis);
	float fA = 0.95; 
	float fB = 1.0-fA;
	kinectFrameRate = (fA*kinectFrameRate) + (fB*FR); 
	kinectPrevFrameMillis = now;
}






