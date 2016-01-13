#ifndef __CROSSHAIR_H__
#define __CROSSHAIR_H__

static const float crosshair[6 * 3] = {
	-0.1, 0.0f, 0.0f,
	0.1f, 0.0f, 0.0f,
	0.0f, 0.0f, -0.1f,
	0.0f, 0.0f, 0.1f,
	0.0f, -0.1f, 0.0f,
	0.0f, 0.1f, 0.0f,
};


static const char crosshair_color[6 * 4] = {
	255, 0, 0, 255,
	0, 0, 0, 255,
	0, 255, 0, 255,
	0, 0, 0, 255,
	0, 0, 255, 255,
	0, 0, 0, 255,
};

#endif
