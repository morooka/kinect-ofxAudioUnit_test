#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxAudioUnit.h"

class ofApp : public ofBaseApp {
public:
	
	void setup();
	void update();
	void draw();
	void exit();
	
	void keyPressed(int key);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
	
	ofxKinect kinect;
	
	ofxCvColorImage colorImg;
	
	ofxCvGrayscaleImage grayImage; // grayscale depth image
	ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
	ofxCvGrayscaleImage grayThreshFar; // the far thresholded image
	
	ofxCvContourFinder contourFinder;
	
	int nearThreshold;
	int farThreshold;
	
	int angle;
    
    ofxAudioUnit compressor;
	ofxAudioUnit delay;
	ofxAudioUnit distortion;
	ofxAudioUnit filter;
	
	ofxAudioUnitFilePlayer source1, source2, source3;
	ofxAudioUnitMixer mixer;
	ofxAudioUnitOutput output;
	
	ofxAudioUnitTap tap1, tap2, tap3;
	ofPolyline wave1, wave2, wave3;
    
    ofxAudioUnit      varispeed,varispeed2,varispeed3;
	ofxAudioUnit      lowpass,lowpass2,lowpass3;
	
	
};
