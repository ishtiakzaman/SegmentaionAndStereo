all : segment stereo render

segment: CImg.h segment.cpp
	g++ -Dcimg_display=0 segment.cpp -o segment -lX11 -lpthread -I. -O3

stereo: CImg.h stereo.cpp
	g++ -Dcimg_display=0 stereo.cpp -o stereo -lX11 -lpthread -I. -O3

render: CImg.h render.cpp
	g++ -Dcimg_display=0 render.cpp -o render -lX11 -lpthread -I. -O3

clean:
	rm segment stereo render