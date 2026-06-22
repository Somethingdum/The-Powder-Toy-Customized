#pragma once
#include "SimulationConfig.h"
#include <memory>

class Simulation;
struct RenderableSimulation;
class RowWorkerPool;

class Air
{
public:
	Simulation & sim;
	int airMode;
	float ambientAirTemp;
	float edgePressure;
	float edgeVelocityX;
	float edgeVelocityY;
	float vorticityCoeff;
	int convectionMode;
	float ovx[YCELLS][XCELLS];
	float ovy[YCELLS][XCELLS];
	float opv[YCELLS][XCELLS];
	float ohv[YCELLS][XCELLS]; // Ambient Heat
	unsigned char bmap_blockair[YCELLS][XCELLS];
	unsigned char bmap_blockairh[YCELLS][XCELLS];
	float kernel[9];
	void make_kernel(void);
	static float vorticity(const RenderableSimulation & sm, int y, int x);
	void update_airh(void);
	void update_air(void);
	void Clear();
	void ClearAirH();
	void Invert();
	void ApproximateBlockAirMaps(Rect<int> targetBlocks);
	Air(Simulation & sim);
	~Air();
private:
	// Lazily created on first update_air(); only the ticking Simulation ever
	// spawns worker threads. Defaults to the performance-core count (see
	// Air.cpp), overridable via the TPT_AIR_THREADS environment variable.
	std::unique_ptr<RowWorkerPool> airPool;
	RowWorkerPool &AirPool();
};
