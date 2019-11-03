#include "ofApp.h"
#include <iostream>

/*
    If you are struggling to get the device to connect ( especially Windows Users )
    please look at the ReadMe: in addons/ofxKinect/README.md
*/

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
	
#ifdef USE_TWO_KINECTS
	kinect2.init();
	kinect2.open();
#endif
	
	colorImg.allocate(kinect.width, kinect.height);
	grayImage.allocate(kinect.width, kinect.height);
	grayThreshNear.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);
	
	nearThreshold = 230;
	farThreshold = 70;
	bThreshWithOpenCV = true;
	
	ofSetFrameRate(60);
	
	// zero the tilt on startup
	angle = 0;
	kinect.setCameraTiltAngle(angle);
	
	// start from the front
	bDrawPointCloud = false;
    
    tileCount = 10;
    shapeSize = 10;
    newShapeSize = shapeSize;
    shapeAngle = 0;
    
    sizeMode = 0;
    
    svg1.load("module_1.svg");
    svg2.load("module_2.svg");
    svg3.load("module_3.svg");
    svg4.load("module_4.svg");
    svg5.load("module_5.svg");
    svg6.load("module_6.svg");
    svg7.load("module_7.svg");

    shapes[0] = svg1;
    shapes[1] = svg2;
    shapes[2] = svg3;
    shapes[3] = svg4;
    shapes[4] = svg5;
    shapes[5] = svg6;
    shapes[6] = svg7;
    
    tileWidth = ofGetWidth() / tileCount;
    tileHeight = ofGetHeight() / tileCount;
    
    maxDist = sqrt(pow(ofGetWidth(), 2) + pow(ofGetHeight(), 2));
    
    iterator = 0;
}

//--------------------------------------------------------------
void ofApp::update() {
	
	ofBackground(100, 100, 100);
	
	kinect.update();
	
	// there is a new frame and we are connected
	if(kinect.isFrameNew()) {
		
		// load grayscale depth image from the kinect source
		grayImage.setFromPixels(kinect.getDepthPixels());
		
		// we do two thresholds - one for the far plane and one for the near plane
		// we then do a cvAnd to get the pixels which are a union of the two thresholds
//		if(bThreshWithOpenCV) {
//			grayThreshNear = grayImage;
//			grayThreshFar = grayImage;
//			grayThreshNear.threshold(nearThreshold, true);
//			grayThreshFar.threshold(farThreshold);
//			cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
//		} else {
//			// or we do it ourselves - show people how they can work with the pixels
//			ofPixels & pix = grayImage.getPixels();
//			int numPixels = pix.size();
//			for(int i = 0; i < numPixels; i++) {
//				if(pix[i] < nearThreshold && pix[i] > farThreshold) {
//					pix[i] = 255;
//				} else {
//					pix[i] = 0;
//				}
//			}
//		}
        // ampersand directly changes what is assigned / does not create new variable
        //loop through y columns first
        
        // below code gets closest pixel
//        int closest = 0;
//
//        ofPixels & pix = grayImage.getPixels();
//        int numPixels = pix.size();
//        for (int y = 0; y < kinect.height; y++) {
//            for (int x = 0; x < kinect.width; x++) {
//                int i = x + y * kinect.width;
//                if(pix[i] < nearThreshold && pix[i] > farThreshold) {
//                    if (pix[i] > closest) {
//                        closest = pix[i];
//                        closestX = x;
//                        closestY = y;
//
//                        closestX = ofMap(closestX, 0, 640, 0, ofGetWindowWidth());
//                        closestY= ofMap(closestY, 0, 480, 0, ofGetWindowHeight());
//
//                        smoothedX += (closestX - smoothedX) * 0.02;
//                        smoothedY += (closestY - smoothedY) * 0.02;
//
//                    }
//                    pix[i] = 255;
//                } else {
//                    pix[i] = 0;
//                }
//            }
//        }
        //below code gets average dist.
        
        int amount = 0;

        ofPixels & pix = grayImage.getPixels();
        int numPixels = pix.size();
        for (int y=0;y<kinect.height;y++){
            for (int x=0; x<kinect.width; x++) {
                int index = x+y*kinect.width;
                if(pix[index] < nearThreshold && pix[index] > farThreshold) {
                    averageX += x;
                    averageY += y;
                    averageX = ofMap(averageX, 0, 640, -200, ofGetWindowWidth());
                    averageY = ofMap(averageY, 0, 480, -200, ofGetWindowHeight());
                    amount ++;
                    pix[index] = 255;
                } else {
                    pix[index] = 0;
                }
            }
        }
            if (amount>0){
                    averageX /= amount;
                    averageY /= amount;
            }
        smoothedX += (averageX - smoothedX) * 0.02;
        smoothedY += (averageY - smoothedY) * 0.02;




        std::cout<<smoothedX<<endl;

		// update the cv images
		grayImage.flagImageChanged();
		
		// find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
		// also, find holes is set to true so we will get interior contours as well....
//        contourFinder.findContours(grayImage, 10, (kinect.width*kinect.height)/2, 40, false);
        contourFinder.findContours(grayImage, 10, (kinect.width*kinect.height)/2, 40, true);

	}
	
#ifdef USE_TWO_KINECTS
	kinect2.update();
#endif
}

//--------------------------------------------------------------
void ofApp::draw() {
	
	ofSetColor(255, 255, 255);
	
	if(bDrawPointCloud) {
		easyCam.begin();
		drawPointCloud();
		easyCam.end();
	} else {
		// draw from the live kinect
//		kinect.drawDepth(10, 10, 400, 300);
//		kinect.draw(420, 10, 400, 300);
//
//		grayImage.draw(10, 320, 400, 300);
//		contourFinder.draw(10, 320, 400, 300);
//        ofPushMatrix();
//        ofTranslate(10, 320);
        ofSetColor(255, 0, 0);
        ofDrawEllipse(smoothedX, smoothedY, 30, 30);
//        ofPopMatrix();
#ifdef USE_TWO_KINECTS
		kinect2.draw(420, 320, 400, 300);
#endif
	}
    ofSetColor(255, 255, 255);
	stringstream reportStream;

    for (int gridY = 0; gridY < tileCount; gridY++) {
        for (int gridX = 0; gridX < tileCount; gridX++) {
            float posX = tileWidth * gridX + tileWidth / 2;
            float posY = tileHeight * gridY + tileWidth / 2;
            
            // calculate angle between mouse position and actual position of the shape
            float angle = atan2(smoothedY - posY, smoothedX - posX) + (shapeAngle * (PI / 180));
            
            if (sizeMode == 0) newShapeSize = shapeSize;
            if (sizeMode == 1) newShapeSize = shapeSize * 1.5 - ofMap(ofDist(smoothedX, smoothedY, posX, posY), 0, 500, 5, shapeSize);
            if (sizeMode == 2) newShapeSize = ofMap(ofDist(smoothedX, smoothedY, posX, posY), 0, 500, 5, shapeSize);
            
            
            ofPushMatrix();
            ofTranslate(posX, posY);
            ofRotateRad(angle);
            ofSetLineWidth(0);
//            ofScale(newShapeSize / 10);
            ofScale(newShapeSize/shapes[iterator].getWidth(), newShapeSize/shapes[iterator].getHeight());
            shapes[iterator].draw();
            ofSetColor(0);
            ofDrawLine(0, 0, newShapeSize, 0);
            ofPopMatrix();
        }
    }

}

void ofApp::drawPointCloud() {
	int w = 640;
	int h = 480;
	ofMesh mesh;
	mesh.setMode(OF_PRIMITIVE_POINTS);
	int step = 2;
	for(int y = 0; y < h; y += step) {
		for(int x = 0; x < w; x += step) {
			if(kinect.getDistanceAt(x, y) > 0) {
				mesh.addColor(kinect.getColorAt(x,y));
				mesh.addVertex(kinect.getWorldCoordinateAt(x, y));
			}
		}
	}
	glPointSize(3);
	ofPushMatrix();
	// the projected points are 'upside down' and 'backwards' 
	ofScale(1, -1, -1);
	ofTranslate(0, 0, -1000); // center the points a bit
	ofEnableDepthTest();
	mesh.drawVertices();
	ofDisableDepthTest();
	ofPopMatrix();
}

//--------------------------------------------------------------
void ofApp::exit() {
	kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();
	
#ifdef USE_TWO_KINECTS
	kinect2.close();
#endif
}

//--------------------------------------------------------------
void ofApp::keyPressed (int key) {
	switch (key) {
		case ' ':
			bThreshWithOpenCV = !bThreshWithOpenCV;
			break;
			
		case'p':
			bDrawPointCloud = !bDrawPointCloud;
			break;
			
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
	}
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button)
{
	
}

void ofApp::keyReleased(int key){
    if (key == '1') iterator = 0;
    if (key == '2') iterator = 1;
    if (key == '3') iterator = 2;
    if (key == '4') iterator = 3;
    if (key == '5') iterator = 4;
    if (key == '6') iterator = 5;
    if (key == '7') iterator = 6;
    
    if (key == OF_KEY_UP) shapeSize += 2;
    if (key == OF_KEY_DOWN) shapeSize -= 2;
    if (key == OF_KEY_LEFT) shapeAngle += 5;
    if (key == OF_KEY_RIGHT) shapeAngle -= 5;
    
    if (key == 'd' || key == 'D') sizeMode = (sizeMode + 1) % 3;
    if (key == 'g' || key == 'G') {
        tileCount += 5;
        if (tileCount > 20) {
            tileCount = 10;
        }
        tileWidth = ofGetWidth() / tileCount;
        tileHeight = ofGetHeight() / tileCount;
    }
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button)
{

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button)
{

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h)
{

}
