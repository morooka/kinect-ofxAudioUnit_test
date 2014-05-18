#include "ofApp.h"



//--------------------------------------------------------------
void ofApp::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);
	
	// enable depth->video image calibration
	kinect.setRegistration(true);
    
	kinect.init();
	//kinect.init(true); // shows infrared instead of RGB video image
	//kinect.init(false, false); // disable video image (faster fps)
	
	kinect.open();		// opens first available kinect
	//kinect.open(1);	// open a kinect by id, starting with 0 (sorted by serial # lexicographically))
	//kinect.open("A00362A08602047A");	// open a kinect using it's unique serial #
	
	// print the intrinsic IR sensor values
	if(kinect.isConnected()) {
		ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
		ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
		ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
		ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
	}
	
	
	colorImg.allocate(kinect.width, kinect.height);
	grayImage.allocate(kinect.width, kinect.height);
	grayThreshNear.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);
	
	nearThreshold = 255;
	farThreshold = 240;
	
	ofSetFrameRate(60);
	
	// zero the tilt on startup
	angle = 0;
	kinect.setCameraTiltAngle(angle);
    
    source1.setFile(ofFilePath::getAbsolutePath("kick.wav"));
	source2.setFile(ofFilePath::getAbsolutePath("snare.wav"));
	source3.setFile(ofFilePath::getAbsolutePath("hats.wav"));
	
    //	Now, let's set up a different effect for each one
	
	distortion = ofxAudioUnit(kAudioUnitType_Effect,
							  kAudioUnitSubType_Distortion);
	
	delay = ofxAudioUnit(kAudioUnitType_Effect,
						 kAudioUnitSubType_Delay);
	
	filter = ofxAudioUnit(kAudioUnitType_Effect,
						  kAudioUnitSubType_LowPassFilter);
    
    varispeed = ofxAudioUnit(kAudioUnitType_FormatConverter,kAudioUnitSubType_Varispeed);
    varispeed2 = ofxAudioUnit(kAudioUnitType_FormatConverter,kAudioUnitSubType_Varispeed);
    varispeed3 = ofxAudioUnit(kAudioUnitType_FormatConverter,kAudioUnitSubType_Varispeed);
    
    lowpass = ofxAudioUnit(kAudioUnitType_Effect,kAudioUnitSubType_LowPassFilter);
    lowpass2 = ofxAudioUnit(kAudioUnitType_Effect,kAudioUnitSubType_LowPassFilter);
    lowpass3 = ofxAudioUnit(kAudioUnitType_Effect,kAudioUnitSubType_LowPassFilter);
    
	source1.connectTo(distortion).connectTo(varispeed).connectTo(lowpass).connectTo(tap1);
	source2.connectTo(delay).connectTo(varispeed2).connectTo(lowpass2).connectTo(tap2);
	source3.connectTo(filter).connectTo(varispeed3).connectTo(lowpass3).connectTo(tap3);
	
	mixer.setInputBusCount(3);
	tap1.connectTo(mixer, 0);
	tap2.connectTo(mixer, 1);
	tap3.connectTo(mixer, 2);
	
	compressor = ofxAudioUnit(kAudioUnitType_Effect,
							  kAudioUnitSubType_DynamicsProcessor);
	
	mixer.connectTo(compressor).connectTo(output);
	
	mixer.setInputVolume(0.5, 2);
	
	output.start();
	
	source1.loop();
	source2.loop();
	source3.loop();
}

//--------------------------------------------------------------
void ofApp::update() {
	
	ofBackground(100, 100, 100);
	
	kinect.update();
	
	if(kinect.isFrameNew()) {
		
		grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
			
			unsigned char * pix = grayImage.getPixels();
			
			int numPixels = grayImage.getWidth() * grayImage.getHeight();
			for(int i = 0; i < numPixels; i++) {
				if(pix[i] < nearThreshold && pix[i] > farThreshold) {
					pix[i] = 255;
				} else {
					pix[i] = 0;
				}
			}
		grayImage.flagImageChanged();
		contourFinder.findContours(grayImage, 10, (kinect.width*kinect.height)/2, 20, false);
        
         if(contourFinder.nBlobs>0){
             float newSpeed = ofMap(contourFinder.blobs[0].centroid.x, 0, ofGetWidth(), 0.8, 1.5, true);   //再生スピード
             AudioUnitSetParameter(varispeed.getUnit(),kVarispeedParam_PlaybackRate,kAudioUnitScope_Global,0,newSpeed,0);
             
             float newCutoff = ofMap(contourFinder.blobs[0].centroid.y, 0, ofGetHeight(), 100, 12000);  //ローパスフィルター
             AudioUnitSetParameter(lowpass.getUnit(),kLowPassParam_CutoffFrequency,kAudioUnitScope_Global,0,newCutoff,0);
         }
        
        if(contourFinder.nBlobs > 1){
            float newSpeed2 = ofMap(contourFinder.blobs[1].centroid.x, 0, ofGetWidth(), 0.8, 1.5, true);   //再生スピード
            AudioUnitSetParameter(varispeed2.getUnit(),kVarispeedParam_PlaybackRate,kAudioUnitScope_Global,0,newSpeed2,0);
            
            float newCutoff2 = ofMap(contourFinder.blobs[1].centroid.y, 0, ofGetHeight(), 100, 12000);  //ローパスフィルター
            AudioUnitSetParameter(lowpass2.getUnit(),kLowPassParam_CutoffFrequency,kAudioUnitScope_Global,0,newCutoff2,0);
        }
        if(contourFinder.nBlobs > 2){
            float newSpeed3 = ofMap(contourFinder.blobs[2].centroid.x, 0, ofGetWidth(), 0.8, 1.5, true);   //再生スピード
            AudioUnitSetParameter(varispeed3.getUnit(),kVarispeedParam_PlaybackRate,kAudioUnitScope_Global,0,newSpeed3,0);
            
            float newCutoff3 = ofMap(contourFinder.blobs[2].centroid.y, 0, ofGetHeight(), 100, 12000);  //ローパスフィルター
            AudioUnitSetParameter(lowpass3.getUnit(),kLowPassParam_CutoffFrequency,kAudioUnitScope_Global,0,newCutoff3,0);
        }
	}
    
    tap1.getLeftWaveform(wave1, ofGetWidth(), ofGetHeight()/3);
	tap2.getLeftWaveform(wave2, ofGetWidth(), ofGetHeight()/3);
	tap3.getLeftWaveform(wave3, ofGetWidth(), ofGetHeight()/3);

	
    
}

//--------------------------------------------------------------
void ofApp::draw() {
	
	ofSetColor(255, 255, 255);
    kinect.draw(0,0);
    glBlendFunc(GL_DST_COLOR, GL_ZERO);
    grayImage.draw(0, 0);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    contourFinder.draw(0,0);
    
    ofPushMatrix();
	{
		ofSetColor(255, 255, 0);
		wave1.draw();
		ofSetColor(255);
		ofDrawBitmapString("Press 'd' for distortion UI", 20, 20);
		
		ofTranslate(0, ofGetHeight()/3);
		
		ofSetColor(0, 255, 255);
		wave2.draw();
		ofSetColor(255);
		ofDrawBitmapString("Press 'e' for delay UI", 20, 20);
		
		ofTranslate(0, ofGetHeight()/3);
		
		ofSetColor(255, 0, 255);
		wave3.draw();
		ofSetColor(255);
		ofDrawBitmapString("Press 'f' for filter UI", 20, 20);
	}
	ofPopMatrix();

}

//--------------------------------------------------------------
void ofApp::exit() {
	kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();
}

//--------------------------------------------------------------
void ofApp::keyPressed (int key) {
	switch (key) {
		case '>':
		case '.':
			farThreshold ++;
			if (farThreshold > 255) farThreshold = 255;
			break;
			
		case '<':
		case ',':
			farThreshold --;
			if (farThreshold < 0) farThreshold = 0;
			break;
			
		case '+':
		case '=':
			nearThreshold ++;
			if (nearThreshold > 255) nearThreshold = 255;
			break;
			
		case '-':
			nearThreshold --;
			if (nearThreshold < 0) nearThreshold = 0;
			break;
			
		case 'w':
			kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
			break;
			
		case 'o':
			kinect.setCameraTiltAngle(angle); // go back to prev tilt
			kinect.open();
			break;
			
		case 'c':
			kinect.setCameraTiltAngle(0); // zero the tilt
			kinect.close();
			break;
			
		case '1':
			kinect.setLed(ofxKinect::LED_GREEN);
			break;
			
		case '2':
			kinect.setLed(ofxKinect::LED_YELLOW);
			break;
			
		case '3':
			kinect.setLed(ofxKinect::LED_RED);
			break;
			
		case '4':
			kinect.setLed(ofxKinect::LED_BLINK_GREEN);
			break;
			
		case '5':
			kinect.setLed(ofxKinect::LED_BLINK_YELLOW_RED);
			break;
			
		case '0':
			kinect.setLed(ofxKinect::LED_OFF);
			break;
			
		case OF_KEY_UP:
			angle++;
			if(angle>30) angle=30;
			kinect.setCameraTiltAngle(angle);
			break;
			
		case OF_KEY_DOWN:
			angle--;
			if(angle<-30) angle=-30;
			kinect.setCameraTiltAngle(angle);
			break;
	}
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button)
{}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button)
{}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h)
{}
