#define LONG_PUSH 1500
#define SHORT_PUSH 500

void initCamera(){
	pinMode(CAM_MODE, OUTPUT);
	digitalWrite(CAM_MODE, LOW);	
}

void pressCameraButton(int button, int duration){
	digitalWrite(button, HIGH);
	delay(duration);
	digitalWrite(button, LOW);
}

/*
	Blocks for 500 ms
*/
inline void startRecording(){
	if(!video_recording){
		pressCameraButton(CAM_MODE, SHORT_PUSH);
		video_recording = true;
	}
}
inline void stopRecording(){
	if(video_recording){
		pressCameraButton(CAM_MODE, SHORT_PUSH);
		video_recording = false;
	}
}