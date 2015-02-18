#pragma once

#include "../scene/material.h"

class CubeMap {

	TextureMap* tMap[6];
	int* kernel;

public:
	CubeMap() : kernel(0) { 
		for (int i = 0; i < 6; i++) tMap[i] = 0;
	}

	void setXposMap(TextureMap* m) {
		if(tMap[0] && tMap[0] != m) delete(tMap[0]);
		if (tMap[0] != m) tMap[0] = m;
	}
	void setXnegMap(TextureMap* m) {
		if(tMap[1] && tMap[1] != m) delete(tMap[1]);
		if (tMap[1] != m) tMap[1] = m;
	}
	void setYposMap(TextureMap* m) {
		if(tMap[2] && tMap[2] != m) delete(tMap[2]);
		if (tMap[2] != m) tMap[2] = m;
	}
	void setYnegMap(TextureMap* m) {
		if(tMap[3] && tMap[3] != m) delete(tMap[3]);
		if (tMap[3] != m) tMap[3] = m;
	}
	void setZposMap(TextureMap* m) {
		if(tMap[4] && tMap[4] != m) delete(tMap[4]);
		if (tMap[4] != m) tMap[4] = m;
	}
	void setZnegMap(TextureMap* m) {
		if(tMap[5] && tMap[5] != m) delete(tMap[5]);
		if (tMap[5] != m) tMap[5] = m;
	}

	Vec3d getColor(ray r) const;

	~CubeMap() {
		for (int i = 0; i < 6; i++) if (tMap[i]) { delete tMap[i]; tMap[i] = 0; }
		if (kernel) delete[] kernel;
	}
};