
#ifndef __SSR_H__
#define __SSR_H__




// SSR message metadata
struct SSRMeta
{
	int				epochTime1s			= 0;
	int				ssrUpdateIntIndex	= -1;
	int				multipleMessage		= 0;
	unsigned int	referenceDatum		= 0;
	unsigned int	provider			= 0;
	unsigned int	solution			= 0;
};

struct SSREph
{
	SSRMeta ssrMeta;
	GTime	t0			= {};
	double	udi			= 0;		///< update interval
	int		iod			= -1;	
	int		iode		= -1;		///< issue of data
	int		iodcrc		= -1;
	double	deph [3]	= {};		///< delta orbit {radial,along,cross} (m)
	double	ddeph[3]	= {};		///< dot delta orbit {radial,along,cross} (m/s)
};

struct SSRClk
{
	SSRMeta ssrMeta	= {};
	GTime	t0		= {};
	double	udi		= 0;			///< update interval
	int		iod		= -1;
	double	dclk[3]	= {};			///< delta clock {c0,c1,c2} (m,m/s,m/s^2)
};

struct SSRHRClk
{
	SSRMeta ssrMeta	= {};
	GTime	t0		= {};
	double	udi		= 0;			///< update interval
	int		iod		= -1;
	double	hrclk	= 0;			///< high-rate clock corection (m)
};

struct SSRUra
{
	GTime	t0	= {};
	double	udi	= 0;				///< update interval
	int		iod	= -1;
	int		ura	= 0;				///< URA indicator
};

struct SSRPhase
{
	int dispBiasConistInd	= -1;
	int MWConistInd			= -1;
	unsigned int nbias		= 0;
	double yawAngle			= 0;
	double yawRate			= 0;
};

struct SSRPhaseCh
{
	unsigned int signalIntInd		= -1;
	unsigned int signalWidIntInd	= -1;
	unsigned int signalDisconCnt	= -1;
};

struct BiasVar
{
	double bias	= 0;				///< biases (m) 
	double var	= 0;				///< biases variance (m^2) 
};


struct SSRBias
{
	SSRMeta		ssrMeta;
	
	GTime	t0	= {};
	double	udi	= 0;				///< update interval
	int		iod	= -1;
	
	map<E_ObsCode, BiasVar> codeBias_map;			
};

struct SSRCodeBias : SSRBias
{
	//just inherit.
};

struct SSRPhasBias : SSRBias
{
	SSRPhase	ssrPhase; 					///< Additional data for SSR phase messages
	map<E_ObsCode,SSRPhaseCh> ssrPhaseChs;	///< Additional data for SSR phase messages, for each channel
};

struct SSRCodeBiasOut : SSRCodeBias
{
	bool canExport	= false;
	bool isSet		= false;
};

struct SSRPhasBiasOut : SSRPhasBias
{
	bool canExport	= false;
	bool isSet		= false;
};

struct SSRClkOut : SSRClk
{
	double  broadcast		= 0;	// (m)
	double  precise			= 0;	// (m)
	bool    isBroadcastSet	= false;
	bool    isPreciseSet	= false;
	bool	canExport		= false;
};

struct SSREphOut : SSREph
{
	Vector3d	broadcast			= Vector3d::Zero();
	Vector3d	broadcastVel		= Vector3d::Zero();
	Vector3d	precise				= Vector3d::Zero();
	Vector3d	nextBroadcast		= Vector3d::Zero();
	Vector3d	nextBroadcastVel	= Vector3d::Zero();
	Vector3d	nextPrecise			= Vector3d::Zero();
	bool    	isBroadcastSet		= false;
	bool    	isPreciseSet		= false;
	bool    	isNextBroadcastSet	= false;
	bool    	isNextPreciseSet	= false;
	bool		canExport			= false;
};	

/* SSR correction type */
struct ssr_t
{
	map<GTime, SSRCodeBias,	std::greater<GTime>>	ssrCodeBias_map;
	map<GTime, SSRPhasBias,	std::greater<GTime>>	ssrPhasBias_map;
	map<GTime, SSRClk,		std::greater<GTime>>	ssrClk_map;
	map<GTime, SSREph,		std::greater<GTime>>	ssrEph_map;
	map<GTime, SSRHRClk,	std::greater<GTime>>	ssrHRClk_map;		//todo aaron missing implementation?
	map<GTime, SSRUra,		std::greater<GTime>>	ssrUra_map;

	int refd_;					///< sat ref datum (0:ITRF,1:regional)
	unsigned char update_;		///<update flag (0:no update,1:update)
};

struct SSROut
{
	SSRPhasBiasOut	ssrPhasBias;
	SSRCodeBiasOut	ssrCodeBias;
	SSRClkOut       ssrClk;
	SSREphOut       ssrEph;
	int             udiNumEpochs		= 0; ///< UDI in epochs
	int             epochsSinceUpdate	= 0;
	int             numObs				= 0; ///< Number of observations for this sat
};

void initSsrOut();

void calcSsrCorrections(
	Trace&				trace,
	KFState&			kfState,
	std::set<SatSys>&	sats,
	GTime				time);

void writeSsrOutToFile(
	int					epochNum,
	std::set<SatSys>	sats);

#endif
