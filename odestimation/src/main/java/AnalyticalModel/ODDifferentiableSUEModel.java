package AnalyticalModel;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.Map.Entry;
import java.util.concurrent.ConcurrentHashMap;
import java.util.stream.Collectors;

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealVector;
import org.apache.log4j.Logger;
import org.matsim.api.core.v01.Id;
import org.matsim.api.core.v01.Scenario;
import org.matsim.api.core.v01.network.Link;
import org.matsim.api.core.v01.network.Network;
import org.matsim.api.core.v01.population.Person;
import org.matsim.api.core.v01.population.Plan;
import org.matsim.api.core.v01.population.Population;
import org.matsim.core.config.Config;
import org.matsim.core.config.ConfigUtils;
import org.matsim.core.network.NetworkUtils;
import org.matsim.core.population.PopulationUtils;
import org.matsim.core.scoring.functions.ScoringParameters;
import org.matsim.core.utils.collections.Tuple;
import org.matsim.pt.transitSchedule.api.Departure;
import org.matsim.pt.transitSchedule.api.TransitLine;
import org.matsim.pt.transitSchedule.api.TransitRoute;
import org.matsim.pt.transitSchedule.api.TransitSchedule;
import org.matsim.vehicles.VehicleCapacity;
import org.matsim.vehicles.VehicleType;
import org.matsim.vehicles.Vehicles;

import core.MapToArray;
import core.ODUtils;
import dynamicTransitRouter.fareCalculators.FareCalculator;

import transitCalculatorsWithFare.FareLink;
import ust.hk.praisehk.metamodelcalibration.analyticalModel.AnalyticalModelLink;
import ust.hk.praisehk.metamodelcalibration.analyticalModel.AnalyticalModelNetwork;
import ust.hk.praisehk.metamodelcalibration.analyticalModel.AnalyticalModelODpair;
import ust.hk.praisehk.metamodelcalibration.analyticalModel.AnalyticalModelRoute;
import ust.hk.praisehk.metamodelcalibration.analyticalModel.AnalyticalModelTransitRoute;
import ust.hk.praisehk.metamodelcalibration.analyticalModel.SUEModelOutput;
import ust.hk.praisehk.metamodelcalibration.analyticalModel.TransitDirectLink;
import ust.hk.praisehk.metamodelcalibration.analyticalModel.TransitLink;
import ust.hk.praisehk.metamodelcalibration.analyticalModel.TransitTransferLink;
import ust.hk.praisehk.metamodelcalibration.analyticalModelImpl.CNLLink;
import ust.hk.praisehk.metamodelcalibration.analyticalModelImpl.CNLNetwork;
import ust.hk.praisehk.metamodelcalibration.analyticalModelImpl.CNLODpairs;
import ust.hk.praisehk.metamodelcalibration.analyticalModelImpl.CNLSUEModel;
import ust.hk.praisehk.metamodelcalibration.analyticalModelImpl.CNLTransitDirectLink;
import ust.hk.praisehk.metamodelcalibration.analyticalModelImpl.CNLTransitRoute;
import ust.hk.praisehk.metamodelcalibration.analyticalModelImpl.CNLTransitTransferLink;
import ust.hk.praisehk.metamodelcalibration.matsimIntegration.SignalFlowReductionGenerator;
import ust.hk.praisehk.metamodelcalibration.measurements.Measurement;
import ust.hk.praisehk.metamodelcalibration.measurements.MeasurementType;
import ust.hk.praisehk.metamodelcalibration.measurements.Measurements;

public class ODDifferentiableSUEModel {
Logger logger = Logger.getLogger(ODDifferentiableSUEModel.class);

private Map<String,Double> consecutiveSUEErrorIncrease=new ConcurrentHashMap<>();
private LinkedHashMap<String,Double> AnalyticalModelInternalParams=new LinkedHashMap<>();
private LinkedHashMap<String,Double> Params=new LinkedHashMap<>();
private LinkedHashMap<String,Tuple<Double,Double>> AnalyticalModelParamsLimit=new LinkedHashMap<>();
private boolean intiializeGradient = true;

private double alphaMSA=1.9;//parameter for decreasing MSA step size
private double gammaMSA=.1;//parameter for decreasing MSA step size

//other Parameters for the Calibration Process
private double tollerance= 1;
private double tolleranceLink=1;
//user input
public boolean calcvehicleSpecificRouteFlow=false;

private Map<String, Tuple<Double,Double>> timeBeans;

//MATSim Input
private Map<String, AnalyticalModelNetwork> networks=new ConcurrentHashMap<>();
private TransitSchedule ts;
private Scenario scenario;
private Population population;
protected Map<String,FareCalculator> fareCalculator=new HashMap<>();

//Used Containers
private Map<String,ArrayList<Double>> beta=new ConcurrentHashMap<>(); //This is related to weighted MSA of the SUE
private Map<String,ArrayList<Double>> error=new ConcurrentHashMap<>();
private Map<String,ArrayList<Double>> error1=new ConcurrentHashMap<>();//This is related to weighted MSA of the SUE

//TimebeanId vs demands map
private Map<String,Map<Id<AnalyticalModelODpair>,Double>> Demand=new HashMap<>();//Holds ODpair based demand
private Map<String,HashMap<Id<AnalyticalModelODpair>,Double>> carDemand=new ConcurrentHashMap<>(); 
private CNLODpairs odPairs;
private Map<String,Map<Id<TransitLink>,TransitLink>> transitLinks=new ConcurrentHashMap<>();
	
private Population lastPopulation;

private Map<String,Map<String,Double>> suPopSpecificParam = new ConcurrentHashMap<>();
//Internal database for the utility, mode choice and utility

private Map<String,Map<Id<AnalyticalModelRoute>,Double>> routeUtilities = new HashMap<>();
private Map<String,Map<Id<AnalyticalModelTransitRoute>,Double>> trRouteUtilities = new HashMap<>();
private Map<String,Map<Id<AnalyticalModelODpair>,Double>> expectedMaximumCarUtility = new HashMap<>();
private Map<String,Map<Id<AnalyticalModelODpair>,Double>> expectedMaximumTrUtility = new HashMap<>();

private Map<String,Map<Id<AnalyticalModelRoute>,Double>> routeProb = new HashMap<>();
private Map<String,Map<Id<AnalyticalModelTransitRoute>,Double>> trRouteProb = new HashMap<>();

private Map<String,Map<Id<AnalyticalModelRoute>,Double>> routeFlow = new HashMap<>();
private Map<String,Map<Id<AnalyticalModelTransitRoute>,Double>> trRouteFlow = new HashMap<>();

private Map<String,Map<Id<AnalyticalModelODpair>,Double>> carProbability = new HashMap<>();
private double linkGradL1NormThreshold = 1000;
//The gradient containers
//time->id->varKey->grad
private Map<Id<AnalyticalModelODpair>,List<Id<AnalyticalModelRoute>>> routeODIncidence = new HashMap<>();
private Map<String,Map<Id<AnalyticalModelODpair>,List<Id<AnalyticalModelTransitRoute>>>> trRouteODIncidence = new HashMap<>();
private Map<Id<Link>, List<Id<AnalyticalModelRoute>>> linkIncidenceMatrix = new HashMap<>();
private Map<String,Map<Id<TransitLink>, List<Id<AnalyticalModelTransitRoute>>>> trLinkIncidenceMatrix = new HashMap<>();
private Map<String,Map<String,List<Id<AnalyticalModelTransitRoute>>>> fareLinkincidenceMatrix = new HashMap<>();
private Set<String> gradientKeys;

//private Map<String,Map<Id<Link>,Map<String,Double>>> linkGradient = new HashMap<>();
private Map<String,Map<Id<Link>,double[]>> linkGradient = new HashMap<>();

//private Map<String,Map<Id<Link>,Map<String,Double>>> linkTTGradient = new HashMap<>();
private Map<String,Map<Id<Link>,double[]>> linkTTGradient = new HashMap<>();

//private Map<String,Map<Id<TransitLink>,Map<String,Double>>> trLinkGradient = new HashMap<>();
private Map<String,Map<Id<TransitLink>,double[]>> trLinkGradient = new HashMap<>();


//private Map<String,Map<Id<TransitLink>,Map<String,Double>>> trLinkTTGradient = new HashMap<>();
private Map<String,Map<Id<TransitLink>,double[]>> trLinkTTGradient = new HashMap<>();

//private Map<String,Map<Id<AnalyticalModelRoute>,Map<String,Double>>> routeFlowGradient = new HashMap<>();
//private Map<String,Map<Id<AnalyticalModelTransitRoute>,Map<String,Double>>> trRouteFlowGradient = new HashMap<>();

private Map<String,Map<Id<AnalyticalModelRoute>,double[]>> routeFlowGradient = new HashMap<>();
private Map<String,Map<Id<AnalyticalModelTransitRoute>,double[]>> trRouteFlowGradient = new HashMap<>();

//private Map<String,Map<String,Map<String,Double>>> fareLinkGradient = new HashMap<>();

private Map<String,Map<String,double[]>> fareLinkGradient = new HashMap<>();

private MapToArray<String> gradientArray;

private Map<String,Map<Id<Link>,Double>> linkVolumeUpdate = new HashMap<>();
private Map<String,Map<Id<TransitLink>,Double>> linkTrVolumeUpdate = new HashMap<>();

private Map<String,Map<Id<AnalyticalModelODpair>,double[]>>odParameterIncidence = new HashMap<>();
private boolean ifODParameterIncidence = true;
//This are needed for output generation 

private double[] gradMultiplier;
private boolean ifGradMultiply = true;
private double maxAbsGrad = .2;
private double minAbsGrad = 0.01;
private double maxAbsL1Norm = 1e150;
private double minAbsL1Norm = 1e-150;

protected Map<String,Map<Id<Link>,Double>> outputLinkTT=new ConcurrentHashMap<>();
protected Map<String,Map<Id<TransitLink>,Double>> outputTrLinkTT=new ConcurrentHashMap<>();
private Map<String,Map<Id<Link>,Double>> totalPtCapacityOnLink=new HashMap<>();
protected Map<String,Map<String,Double>>MTRCount=new ConcurrentHashMap<>();
//All the parameters name
//They are kept public to make it easily accessible as they are final they can not be modified

private boolean emptyMeasurements;

private Measurements measurementsToUpdate;




public static final String BPRalphaName="BPRalpha";
public static final String BPRbetaName="BPRbeta";
public static final String LinkMiuName="LinkMiu";
public static final String ModeMiuName="ModeMiu";
public static final String TransferalphaName="Transferalpha";
public static final String TransferbetaName="Transferbeta";
	

private void defaultParameterInitiation(Config config){
	this.AnalyticalModelInternalParams.put(CNLSUEModel.LinkMiuName, 1.);
	this.AnalyticalModelInternalParams.put(CNLSUEModel.ModeMiuName, 1.);
	this.AnalyticalModelInternalParams.put(CNLSUEModel.BPRalphaName, 0.15);
	this.AnalyticalModelInternalParams.put(CNLSUEModel.BPRbetaName, 4.);
	this.AnalyticalModelInternalParams.put(CNLSUEModel.TransferalphaName, 0.5);
	this.AnalyticalModelInternalParams.put(CNLSUEModel.TransferbetaName, 1.1);
	this.loadAnalyticalModelInternalPamamsLimit();
	this.Params.put(CNLSUEModel.CapacityMultiplierName, 1.0);
}

protected void loadAnalyticalModelInternalPamamsLimit() {
	this.AnalyticalModelParamsLimit.put(CNLSUEModel.LinkMiuName, new Tuple<Double,Double>(0.0075,0.25));
	this.AnalyticalModelParamsLimit.put(CNLSUEModel.ModeMiuName, new Tuple<Double,Double>(0.01,0.5));
	this.AnalyticalModelParamsLimit.put(CNLSUEModel.BPRalphaName, new Tuple<Double,Double>(0.10,4.));
	this.AnalyticalModelParamsLimit.put(CNLSUEModel.BPRbetaName, new Tuple<Double,Double>(1.,15.));
	this.AnalyticalModelParamsLimit.put(CNLSUEModel.TransferalphaName, new Tuple<Double,Double>(0.25,5.));
	this.AnalyticalModelParamsLimit.put(CNLSUEModel.TransferbetaName, new Tuple<Double,Double>(0.75,4.));
	
}

public ODDifferentiableSUEModel(Map<String, Tuple<Double, Double>> timeBean,Config config) {
	this.timeBeans=timeBean;
	//this.defaultParameterInitiation(null);
	for(String timeId:this.timeBeans.keySet()) {
		this.transitLinks.put(timeId, new HashMap<Id<TransitLink>, TransitLink>());
	
		
		this.carDemand.put(timeId, new HashMap<Id<AnalyticalModelODpair>, Double>());
		this.carProbability.put(timeId, new ConcurrentHashMap<>());
		this.routeUtilities.put(timeId,new ConcurrentHashMap<>());
		this.trRouteUtilities.put(timeId, new ConcurrentHashMap<>());
		this.routeFlow.put(timeId, new ConcurrentHashMap<>());
		this.trRouteFlow.put(timeId, new ConcurrentHashMap<>());
		this.routeProb.put(timeId, new ConcurrentHashMap<>());
		this.trRouteProb.put(timeId, new ConcurrentHashMap<>());
		this.expectedMaximumCarUtility.put(timeId, new ConcurrentHashMap<>());
		this.expectedMaximumTrUtility.put(timeId, new ConcurrentHashMap<>());
		this.error.put(timeId, new ArrayList<>());
		this.beta.put(timeId, new ArrayList<>());
		this.error1.put(timeId, new ArrayList<>());
		
		//For result recording
		outputLinkTT.put(timeId, new HashMap<>());
		outputTrLinkTT.put(timeId, new HashMap<>());
		this.totalPtCapacityOnLink.put(timeId, new HashMap<>());
		this.MTRCount.put(timeId, new ConcurrentHashMap<>());
	}
	if(config==null) config = ConfigUtils.createConfig();
	this.defaultParameterInitiation(config);
	logger.info("Model created.");
}

/**
 * This method overlays transit vehicles on the road network
 * @param network
 * @param Schedule
 */
public void performTransitVehicleOverlay(AnalyticalModelNetwork network, TransitSchedule schedule,Vehicles vehicles,String timeBeanId) {
	for(TransitLine tl:schedule.getTransitLines().values()) {
		for(TransitRoute tr:tl.getRoutes().values()) {
			ArrayList<Id<Link>> links=new ArrayList<>(tr.getRoute().getLinkIds());
			for(Departure d:tr.getDepartures().values()) {
				if(d.getDepartureTime()>this.timeBeans.get(timeBeanId).getFirst() && d.getDepartureTime()<=this.timeBeans.get(timeBeanId).getSecond()) {
					for(Id<Link> linkId:links) {
						CNLLink link=((CNLLink)network.getLinks().get(linkId));
						VehicleType vt=vehicles.getVehicles().get(d.getVehicleId()).getType();
						link.addLinkTransitVolume(vt.getPcuEquivalents());
						if(this.calcvehicleSpecificRouteFlow)link.addVehicleSpecificVolume(1, vt.getId(), true);
						Double oldCap=this.totalPtCapacityOnLink.get(timeBeanId).get(linkId);
						VehicleCapacity cap=vehicles.getVehicles().get(d.getVehicleId()).getType().getCapacity();
						if(oldCap!=null) {
							this.totalPtCapacityOnLink.get(timeBeanId).put(linkId, oldCap+(cap.getSeats()+cap.getStandingRoom()));
						}else {
							this.totalPtCapacityOnLink.get(timeBeanId).put(linkId, (double) cap.getSeats()+cap.getStandingRoom());
						}
						}
				}
			}
		}
	}
	logger.info("Completed transit vehicle overlay.");
}

public void generateRoutesAndOD(Population population,Network network,Network odNetwork, TransitSchedule transitSchedule,
		Scenario scenario,Map<String,FareCalculator> fareCalculator) {
	this.scenario = scenario;
	this.population = population;
	
	//System.out.println("");
	this.odPairs = new CNLODpairs(network,population,transitSchedule,scenario,this.timeBeans);
//	Config odConfig=ConfigUtils.createConfig();
//	odConfig.network().setInputFile("data/odNetwork.xml");

	//This is for creating ODpairs based on TPUSBs
	this.odPairs.generateODpairsetSubPop(odNetwork);//This network has priority over the constructor network. This allows to use a od pair specific network 
	this.odPairs.generateOdSpecificRouteKeys();
	this.odPairs.generateRouteandLinkIncidence(0.);
	SignalFlowReductionGenerator sg=new SignalFlowReductionGenerator(scenario);
	for(String s:this.timeBeans.keySet()) {
		this.networks.put(s, new CNLNetwork(network,sg));
		this.performTransitVehicleOverlay(this.networks.get(s),
				transitSchedule,scenario.getTransitVehicles(),s);
		this.transitLinks.put(s,this.odPairs.getTransitLinks(s));
	}
	this.fareCalculator = fareCalculator;
	this.ts = transitSchedule;
	//this.population.getPersons().values().forEach(p->p.getPlans().clear());
	for(String timeBeanId:this.timeBeans.keySet()) {
		this.consecutiveSUEErrorIncrease.put(timeBeanId, 0.);
		this.Demand.put(timeBeanId, new HashMap<>(this.odPairs.getdemand(timeBeanId)));
		for(Id<AnalyticalModelODpair> odId:this.Demand.get(timeBeanId).keySet()) {
			double totalDemand=this.Demand.get(timeBeanId).get(odId);
			AnalyticalModelODpair odpair = this.odPairs.getODpairset().get(odId);
			this.carDemand.get(timeBeanId).put(odId, 0.5*totalDemand);
			
			if(odpair.getSubPopulation().contains("GV")) {
				this.carDemand.get(timeBeanId).put(odId, totalDemand); 
			}
			//System.out.println();
		}
		
	}
	
	int agentTrip=0;
	int matsimTrip=0;
	int agentDemand=0;
	for(AnalyticalModelODpair odPair:this.odPairs.getODpairset().values()) {
		agentTrip+=odPair.getAgentCounter();
		for(String s:odPair.getTimeBean().keySet()) {
			agentDemand+=odPair.getDemand().get(s);
		}
		
	}
	System.out.println("Demand total = "+agentDemand);
	System.out.println("Total Agent Trips = "+agentTrip);
	this.createLinkRouteIncidence();
}

private void createLinkRouteIncidence(){
	this.timeBeans.keySet().forEach(t->{
		this.trLinkIncidenceMatrix.put(t, new HashMap<>());
		this.fareLinkincidenceMatrix.put(t, new HashMap<>());
		this.odPairs.getTransitLinks(t).keySet().forEach(linkId->{
			this.trLinkIncidenceMatrix.get(t).put(linkId, new ArrayList<>());
		});
	});
	this.odPairs.getODpairset().entrySet().forEach(od->{
		od.getValue().getLinkIncidence().entrySet().forEach(linkInd->{
			List<Id<AnalyticalModelRoute>> routeIds = new ArrayList<>();
			linkInd.getValue().forEach(r->{
				routeIds.add(r.getRouteId());
				
			});
			if(this.linkIncidenceMatrix.containsKey(linkInd.getKey())) {
				this.linkIncidenceMatrix.get(linkInd.getKey()).addAll(routeIds);
			}else {
				this.linkIncidenceMatrix.put(linkInd.getKey(), routeIds);
			}
			List<Id<AnalyticalModelRoute>>routes = new ArrayList<>();
			od.getValue().getRoutes().forEach(r->routes.add(r.getRouteId()));
			this.routeODIncidence.put(od.getKey(), routes);
		});
		this.timeBeans.keySet().forEach(t->{
			Map<Id<TransitLink>,List<Id<AnalyticalModelTransitRoute>>> timeRoutes = this.trLinkIncidenceMatrix.get(t);
			Map<String, List<Id<AnalyticalModelTransitRoute>>> fareLinkTimeRoutes = this.fareLinkincidenceMatrix.get(t);
			od.getValue().getTrLinkIncidence().entrySet().forEach(linkInd->{				
				if(timeRoutes.containsKey(linkInd.getKey())) {
					List<Id<AnalyticalModelTransitRoute>> routeIds = new ArrayList<>();
					linkInd.getValue().forEach(r->{
						routeIds.add(r.getTrRouteId());
					});
					timeRoutes.get(linkInd.getKey()).addAll(routeIds);
				}
				
			});
			this.trRouteODIncidence.compute(t, (k,v)->v==null?v=new HashMap<>():v);
			this.trRouteODIncidence.get(t).put(od.getKey(), new ArrayList<>());
			if(od.getValue().getTrRoutes(t)!=null) {
			od.getValue().getTrRoutes(t).forEach(r->{
				r.getFareLinks().forEach(fl->{
					if(!fareLinkTimeRoutes.containsKey(fl.toString())){
						fareLinkTimeRoutes.put(fl.toString(), new ArrayList<>());	
					}
					fareLinkTimeRoutes.get(fl.toString()).add(r.getTrRouteId());
				});
				this.trRouteODIncidence.get(t).get(od.getKey()).add(r.getTrRouteId());
			});
			}
		
		});
	});
	
}
/**
 * This will not deal with any param containing sub population name or All
 * This hampers in time so currently is turned off
 * @param params
 * @param subPopulation
 * @param config
 * @return
 */
private LinkedHashMap<String,Double> handleBasicParams(LinkedHashMap<String,Double> oldparams, String subPopulation, Config config){
	LinkedHashMap<String,Double> params = new LinkedHashMap<>(oldparams);
	// Handle the original params first
//	for(String s:params.keySet()) {
//		if(subPopulation!=null && (s.contains(subPopulation)||s.contains("All"))) {
//			newParams.put(s.split(" ")[1],params.get(s));
//		}else if (subPopulation == null) {
//			newParams.put(s, params.get(s));
//		}else if(subPopulation!=null) {//this will allow the unknown param to enter
//			newParams.put(s, params.get(s));
//		}
//	}
	if(!this.suPopSpecificParam.containsKey(subPopulation)) {
		LinkedHashMap<String,Double> newParams = new LinkedHashMap<>();
		ScoringParameters scParam = new ScoringParameters.Builder(config.planCalcScore(), config.planCalcScore().getScoringParameters(subPopulation), config.scenario()).build();
		
		newParams.compute(CNLSUEModel.MarginalUtilityofTravelCarName,(k,v)->v==null?scParam.modeParams.get("car").marginalUtilityOfTraveling_s*3600:v);
		newParams.compute(CNLSUEModel.MarginalUtilityofDistanceCarName, (k,v)->v==null?scParam.modeParams.get("car").marginalUtilityOfDistance_m:v);
		newParams.compute(CNLSUEModel.MarginalUtilityofMoneyName, (k,v)->v==null?scParam.marginalUtilityOfMoney:v);
		newParams.compute(CNLSUEModel.DistanceBasedMoneyCostCarName, (k,v)->v==null?scParam.modeParams.get("car").monetaryDistanceCostRate:v);
		newParams.compute(CNLSUEModel.MarginalUtilityofTravelptName, (k,v)->v==null?scParam.modeParams.get("pt").marginalUtilityOfTraveling_s*3600:v);
		newParams.compute(CNLSUEModel.MarginalUtilityOfDistancePtName, (k,v)->v==null?scParam.modeParams.get("pt").marginalUtilityOfDistance_m:v);
		newParams.compute(CNLSUEModel.MarginalUtilityofWaitingName, (k,v)->v==null?scParam.marginalUtilityOfWaitingPt_s*3600:v);
		newParams.compute(CNLSUEModel.UtilityOfLineSwitchName, (k,v)->v==null?scParam.utilityOfLineSwitch:v);
		newParams.compute(CNLSUEModel.MarginalUtilityOfWalkingName, (k,v)->v==null?scParam.modeParams.get("walk").marginalUtilityOfTraveling_s*3600:v);
		newParams.compute(CNLSUEModel.DistanceBasedMoneyCostWalkName, (k,v)->v==null?scParam.modeParams.get("walk").monetaryDistanceCostRate:v);
		newParams.compute(CNLSUEModel.ModeConstantCarName, (k,v)->v==null?scParam.modeParams.get("car").constant:v);
		newParams.compute(CNLSUEModel.ModeConstantPtname, (k,v)->v==null?scParam.modeParams.get("pt").constant:v);
		newParams.compute(CNLSUEModel.MarginalUtilityofPerformName, (k,v)->v==null?scParam.marginalUtilityOfPerforming_s*3600:v);
		
		newParams.compute(CNLSUEModel.CapacityMultiplierName, (k,v)->v==null?config.qsim().getFlowCapFactor():v);
		this.suPopSpecificParam.put(subPopulation, newParams);
		
	}
	this.suPopSpecificParam.get(subPopulation).entrySet().forEach(pp->{
		params.compute(pp.getKey(), (k,v)->v==null?pp.getValue():v);
	});
	return params;
}

public Measurements perFormSUE(LinkedHashMap<String, Double> params,Measurements originalMeasurements) {
	this.Demand = ODUtils.applyODPairMultiplier(this.Demand, params,this.odPairs.getODpairset());
	for(String timeBeanId:this.timeBeans.keySet()) {
		this.consecutiveSUEErrorIncrease.put(timeBeanId, 0.);
		//this.Demand.put(timeBeanId, new HashMap<>(this.odPairs.getdemand(timeBeanId)));
		for(Id<AnalyticalModelODpair> odId:this.Demand.get(timeBeanId).keySet()) {
			double totalDemand=this.Demand.get(timeBeanId).get(odId);
			AnalyticalModelODpair odpair = this.odPairs.getODpairset().get(odId);
			this.carDemand.get(timeBeanId).put(odId, 0.5*totalDemand);
			
			if(odpair.getSubPopulation().contains("GV")) {
				this.carDemand.get(timeBeanId).put(odId, totalDemand); 
			}
			//System.out.println();
		}
		
	}
	return this.performAssignment(params, this.AnalyticalModelInternalParams,originalMeasurements);
}

public Measurements perFormSUE(LinkedHashMap<String, Double> params,Measurements originalMeasurements,boolean calcGrad) {
	this.Demand = ODUtils.applyODPairMultiplier(this.Demand, params,this.odPairs.getODpairset());
	for(String timeBeanId:this.timeBeans.keySet()) {
		this.consecutiveSUEErrorIncrease.put(timeBeanId, 0.);
		//this.Demand.put(timeBeanId, new HashMap<>(this.odPairs.getdemand(timeBeanId)));
		for(Id<AnalyticalModelODpair> odId:this.Demand.get(timeBeanId).keySet()) {
			double totalDemand=this.Demand.get(timeBeanId).get(odId);
			AnalyticalModelODpair odpair = this.odPairs.getODpairset().get(odId);
			this.carDemand.get(timeBeanId).put(odId, 0.5*totalDemand);
			
			if(odpair.getSubPopulation().contains("GV")) {
				this.carDemand.get(timeBeanId).put(odId, totalDemand); 
			}
			//System.out.println();
		}
		
	}
	return this.performAssignment(params, this.AnalyticalModelInternalParams,originalMeasurements,calcGrad);
}

private Measurements performAssignment(LinkedHashMap<String,Double> params, LinkedHashMap<String,Double> anaParams, Measurements originalMeasurements) {
	Measurements measurementsToUpdate = null;
	
	SUEModelOutput flow = this.performAssignment(params, anaParams);
	
	if(originalMeasurements==null) {//for now we just add the fare link and link volume for a null measurements
		this.emptyMeasurements=true;
		measurementsToUpdate=Measurements.createMeasurements(this.timeBeans);
		//create and insert link volume measurement
		for(Entry<String, Map<Id<Link>, Double>> timeFlow:flow.getLinkVolume().entrySet()) {
			for(Entry<Id<Link>, Double> link:timeFlow.getValue().entrySet()) {
				Id<Measurement> mid = Id.create(link.getKey().toString(), Measurement.class);
				if(measurementsToUpdate.getMeasurements().containsKey(mid)) {
					measurementsToUpdate.getMeasurements().get(mid).putVolume(timeFlow.getKey(), link.getValue());
				}else {
					measurementsToUpdate.createAnadAddMeasurement(mid.toString(), MeasurementType.linkVolume);
					List<Id<Link>> links = new ArrayList<>();
					links.add(link.getKey());
					measurementsToUpdate.getMeasurements().get(mid).setAttribute(Measurement.linkListAttributeName, links);
					measurementsToUpdate.getMeasurements().get(mid).putVolume(timeFlow.getKey(), link.getValue());
				}
			}
		}
		
		for(Entry<String, Map<String, Double>> timeFlow:flow.getFareLinkVolume().entrySet()) {
			for(Entry<String, Double> link:timeFlow.getValue().entrySet()) {
				Id<Measurement> mid = Id.create(link.getKey().toString(), Measurement.class);
				if(measurementsToUpdate.getMeasurements().containsKey(mid)) {
					measurementsToUpdate.getMeasurements().get(mid).putVolume(timeFlow.getKey(), link.getValue());
				}else {
					measurementsToUpdate.createAnadAddMeasurement(mid.toString(), MeasurementType.fareLinkVolume);
					measurementsToUpdate.getMeasurements().get(mid).setAttribute(Measurement.FareLinkAttributeName, new FareLink(link.getKey()));
					measurementsToUpdate.getMeasurements().get(mid).putVolume(timeFlow.getKey(), link.getValue());
				}
			}
		}
	}else {
		measurementsToUpdate=originalMeasurements.clone();
		measurementsToUpdate.resetMeasurements();
		measurementsToUpdate.updateMeasurements(flow, null, null);
	}
	return measurementsToUpdate;
}

private Measurements performAssignment(LinkedHashMap<String,Double> params, LinkedHashMap<String,Double> anaParams, Measurements originalMeasurements,boolean calcGrad) {
	Measurements measurementsToUpdate = null;
	
	SUEModelOutput flow = this.performAssignment(params, anaParams,calcGrad);
	
	if(originalMeasurements==null) {//for now we just add the fare link and link volume for a null measurements
		this.emptyMeasurements=true;
		measurementsToUpdate=Measurements.createMeasurements(this.timeBeans);
		//create and insert link volume measurement
		for(Entry<String, Map<Id<Link>, Double>> timeFlow:flow.getLinkVolume().entrySet()) {
			for(Entry<Id<Link>, Double> link:timeFlow.getValue().entrySet()) {
				Id<Measurement> mid = Id.create(link.getKey().toString(), Measurement.class);
				if(measurementsToUpdate.getMeasurements().containsKey(mid)) {
					measurementsToUpdate.getMeasurements().get(mid).putVolume(timeFlow.getKey(), link.getValue());
				}else {
					measurementsToUpdate.createAnadAddMeasurement(mid.toString(), MeasurementType.linkVolume);
					List<Id<Link>> links = new ArrayList<>();
					links.add(link.getKey());
					measurementsToUpdate.getMeasurements().get(mid).setAttribute(Measurement.linkListAttributeName, links);
					measurementsToUpdate.getMeasurements().get(mid).putVolume(timeFlow.getKey(), link.getValue());
				}
			}
		}
		
		for(Entry<String, Map<String, Double>> timeFlow:flow.getFareLinkVolume().entrySet()) {
			for(Entry<String, Double> link:timeFlow.getValue().entrySet()) {
				Id<Measurement> mid = Id.create(link.getKey().toString(), Measurement.class);
				if(measurementsToUpdate.getMeasurements().containsKey(mid)) {
					measurementsToUpdate.getMeasurements().get(mid).putVolume(timeFlow.getKey(), link.getValue());
				}else {
					measurementsToUpdate.createAnadAddMeasurement(mid.toString(), MeasurementType.fareLinkVolume);
					measurementsToUpdate.getMeasurements().get(mid).setAttribute(Measurement.FareLinkAttributeName, new FareLink(link.getKey()));
					measurementsToUpdate.getMeasurements().get(mid).putVolume(timeFlow.getKey(), link.getValue());
				}
			}
		}
	}else {
		measurementsToUpdate=originalMeasurements.clone();
		measurementsToUpdate.resetMeasurements();
		measurementsToUpdate.updateMeasurements(flow, null, null);
	}
	return measurementsToUpdate;
}

private SUEModelOutput performAssignment( LinkedHashMap<String,Double> params, LinkedHashMap<String,Double> anaParams) {
	SUEModelOutput flow = new SUEModelOutput(new HashMap<>(),new HashMap<>(),new HashMap<>(),new HashMap<>(),new HashMap<>());
	//this.resetCarDemand();
	for(String timeId:this.timeBeans.keySet()) {
		SUEModelOutput flowOut = this.singleTimeBeanTA(params, anaParams, timeId);
		
		flow.getLinkVolume().putAll(flowOut.getLinkVolume());
		flow.getLinkTravelTime().putAll(flowOut.getLinkTravelTime());
		flow.getLinkTransitVolume().putAll(flowOut.getLinkTransitVolume());
		flow.getTrLinkTravelTime().putAll(flowOut.getTrLinkTravelTime());
		flow.getFareLinkVolume().putAll(flowOut.getFareLinkVolume());
	}
	return flow;
}

private SUEModelOutput performAssignment( LinkedHashMap<String,Double> params, LinkedHashMap<String,Double> anaParams,boolean calcGradient) {
	SUEModelOutput flow = new SUEModelOutput(new HashMap<>(),new HashMap<>(),new HashMap<>(),new HashMap<>(),new HashMap<>());
	//this.resetCarDemand();
	for(String timeId:this.timeBeans.keySet()) {
		SUEModelOutput flowOut = this.singleTimeBeanTA(params, anaParams, timeId,calcGradient);
		
		flow.getLinkVolume().putAll(flowOut.getLinkVolume());
		flow.getLinkTravelTime().putAll(flowOut.getLinkTravelTime());
		flow.getLinkTransitVolume().putAll(flowOut.getLinkTransitVolume());
		flow.getTrLinkTravelTime().putAll(flowOut.getTrLinkTravelTime());
		flow.getFareLinkVolume().putAll(flowOut.getFareLinkVolume());
	}
	return flow;
}

public SUEModelOutput singleTimeBeanTA(LinkedHashMap<String, Double> params,LinkedHashMap<String,Double> anaParams,String timeBeanId) {
	Map<Id<TransitLink>, Double> linkTransitVolume =null;
	Map<Id<Link>,Double> linkCarVolume = null;
	Map<String,Map<String,Double>> fareLinkVolume = new HashMap<>();
	fareLinkVolume.put(timeBeanId, new HashMap<>());
	boolean shouldStop=false;
//	boolean firstTimeGradCalc = true;
	for(int i=1;i<70;i++) {
		//for(this.car)
		//ConcurrentHashMap<String,HashMap<Id<CNLODpair>,Double>>demand=this.Demand;
		linkCarVolume=this.performCarNetworkLoading(timeBeanId,i,params,anaParams);
		linkTransitVolume=this.performTransitNetworkLoading(timeBeanId,i,params,anaParams);
		System.out.println("Finished network loading.");
		//System.out.println("GB: " + (double) (Runtime.getRuntime().totalMemory() - Runtime.getRuntime().freeMemory()) / (1024*1024*1024));
		shouldStop=this.CheckConvergence(linkCarVolume, linkTransitVolume, this.tollerance, timeBeanId,i);
		///the beta should already be calculated by this line. 
//		double updateRatio = 1/this.beta.get(timeBeanId).get(i-1);
//		if(updateRatio>0.20 || i<20) {
//		//	if(firstTimeGradCalc) {
//				this.caclulateGradient(timeBeanId, i, params, anaParams,true);
//				firstTimeGradCalc = false;
//			}else {
		this.caclulateGradient(timeBeanId, i, params, anaParams);
		//	}
//		}
		System.out.println("Finished gradient Calculation");
		
		this.UpdateLinkVolume(linkCarVolume, linkTransitVolume, i, timeBeanId);
		if(i==1 && shouldStop==true) {
			boolean demandEmpty=true;
			for(AnalyticalModelODpair od:this.odPairs.getODpairset().values()) {
				if(od.getDemand().get(timeBeanId)!=0) {
					demandEmpty=false;
					break;
				}
			}
			if(!demandEmpty) {
				System.out.println("The model cannot converge on first iteration!!!");
			}
		}
		
		if(shouldStop) {
//			//collect travel time
//			if(this.measurementsToUpdate!=null) {
//				List<Measurement>ms= this.measurementsToUpdate.getMeasurementsByType().get(MeasurementType.linkTravelTime);
//				for(Measurement m:ms) {
//					if(m.getVolumes().containsKey(timeBeanId)) {
//						m.putVolume(timeBeanId, ((CNLLink)this.networks.get(timeBeanId).getLinks().get(((ArrayList<Id<Link>>)m.getAttribute(Measurement.linkListAttributeName)).get(0))).getLinkTravelTime(this.timeBeans.get(timeBeanId),
//						params, anaParams));
//					}
//				}
//			}
//			//collect travel time for transit
//			for(TransitLink link:this.transitLinks.get(timeBeanId).values()) {
//				if(link instanceof TransitDirectLink) {
//					this.outputTrLinkTT.get(timeBeanId).put(link.getTrLinkId(), 
//							((TransitDirectLink)link).getLinkTravelTime(this.networks.get(timeBeanId),this.timeBeans.get(timeBeanId),
//									params, anaParams));
//				}else {
//					this.outputTrLinkTT.get(timeBeanId).put(link.getTrLinkId(), 
//							((TransitTransferLink)link).getWaitingTime(anaParams,this.networks.get(timeBeanId)));
//				}
//				
//			}
			this.fareLinkincidenceMatrix.get(timeBeanId).entrySet().stream().forEach(fl->{
				double flow = 0;
				for(Id<AnalyticalModelTransitRoute> trRoute:fl.getValue()){
					flow+=this.trRouteFlow.get(timeBeanId).get(trRoute);
				}
				fareLinkVolume.get(timeBeanId).put(fl.getKey(),flow);
			});
			this.scaleBackGradients();
			break;
			
			}
		this.performModalSplit(params, anaParams, timeBeanId);
		
		
	}
	Map<String,Map<Id<Link>,Double>>linkVolume = new HashMap<>();
	linkVolume.put(timeBeanId, linkCarVolume);
	
	Map<String,Map<Id<TransitLink>,Double>>linkTrVolume = new HashMap<>();
	linkTrVolume.put(timeBeanId, linkTransitVolume);
	SUEModelOutput flow = new SUEModelOutput(linkVolume, linkTrVolume, outputLinkTT, outputTrLinkTT, fareLinkVolume);
	return flow;
	
}

public SUEModelOutput singleTimeBeanTA(LinkedHashMap<String, Double> params,LinkedHashMap<String,Double> anaParams,String timeBeanId, boolean calcGrad) {
	Map<Id<TransitLink>, Double> linkTransitVolume =null;
	Map<Id<Link>,Double> linkCarVolume = null;
	Map<String,Map<String,Double>> fareLinkVolume = new HashMap<>();
	fareLinkVolume.put(timeBeanId, new HashMap<>());
	boolean shouldStop=false;
//	boolean firstTimeGradCalc = true;
	for(int i=1;i<70;i++) {
		//for(this.car)
		//ConcurrentHashMap<String,HashMap<Id<CNLODpair>,Double>>demand=this.Demand;
		linkCarVolume=this.performCarNetworkLoading(timeBeanId,i,params,anaParams);
		linkTransitVolume=this.performTransitNetworkLoading(timeBeanId,i,params,anaParams);
		System.out.println("Finished network loading.");
		//System.out.println("GB: " + (double) (Runtime.getRuntime().totalMemory() - Runtime.getRuntime().freeMemory()) / (1024*1024*1024));
		shouldStop=this.CheckConvergence(linkCarVolume, linkTransitVolume, this.tollerance, timeBeanId,i);
		///the beta should already be calculated by this line. 
//		double updateRatio = 1/this.beta.get(timeBeanId).get(i-1);
//		if(updateRatio>0.20 || i<20) {
//		//	if(firstTimeGradCalc) {
//				this.caclulateGradient(timeBeanId, i, params, anaParams,true);
//				firstTimeGradCalc = false;
//			}else {
		if(calcGrad) {
			this.caclulateGradient(timeBeanId, i, params, anaParams);
		}
		//	}
//		}
		System.out.println("Finished gradient Calculation");
		
		this.UpdateLinkVolume(linkCarVolume, linkTransitVolume, i, timeBeanId);
		if(i==1 && shouldStop==true) {
			boolean demandEmpty=true;
			for(AnalyticalModelODpair od:this.odPairs.getODpairset().values()) {
				if(od.getDemand().get(timeBeanId)!=0) {
					demandEmpty=false;
					break;
				}
			}
			if(!demandEmpty) {
				System.out.println("The model cannot converge on first iteration!!!");
			}
		}
		
		if(shouldStop) {
//			//collect travel time
//			if(this.measurementsToUpdate!=null) {
//				List<Measurement>ms= this.measurementsToUpdate.getMeasurementsByType().get(MeasurementType.linkTravelTime);
//				for(Measurement m:ms) {
//					if(m.getVolumes().containsKey(timeBeanId)) {
//						m.putVolume(timeBeanId, ((CNLLink)this.networks.get(timeBeanId).getLinks().get(((ArrayList<Id<Link>>)m.getAttribute(Measurement.linkListAttributeName)).get(0))).getLinkTravelTime(this.timeBeans.get(timeBeanId),
//						params, anaParams));
//					}
//				}
//			}
//			//collect travel time for transit
//			for(TransitLink link:this.transitLinks.get(timeBeanId).values()) {
//				if(link instanceof TransitDirectLink) {
//					this.outputTrLinkTT.get(timeBeanId).put(link.getTrLinkId(), 
//							((TransitDirectLink)link).getLinkTravelTime(this.networks.get(timeBeanId),this.timeBeans.get(timeBeanId),
//									params, anaParams));
//				}else {
//					this.outputTrLinkTT.get(timeBeanId).put(link.getTrLinkId(), 
//							((TransitTransferLink)link).getWaitingTime(anaParams,this.networks.get(timeBeanId)));
//				}
//				
//			}
			this.fareLinkincidenceMatrix.get(timeBeanId).entrySet().stream().forEach(fl->{
				double flow = 0;
				for(Id<AnalyticalModelTransitRoute> trRoute:fl.getValue()){
					flow+=this.trRouteFlow.get(timeBeanId).get(trRoute);
				}
				fareLinkVolume.get(timeBeanId).put(fl.getKey(),flow);
			});
			this.scaleBackGradients();
			break;
			
			}
		this.performModalSplit(params, anaParams, timeBeanId);
		
		
	}
	Map<String,Map<Id<Link>,Double>>linkVolume = new HashMap<>();
	linkVolume.put(timeBeanId, linkCarVolume);
	
	Map<String,Map<Id<TransitLink>,Double>>linkTrVolume = new HashMap<>();
	linkTrVolume.put(timeBeanId, linkTransitVolume);
	SUEModelOutput flow = new SUEModelOutput(linkVolume, linkTrVolume, outputLinkTT, outputTrLinkTT, fareLinkVolume);
	return flow;
	
}
protected HashMap<Id<Link>,Double> NetworkLoadingCarSingleOD(Id<AnalyticalModelODpair> ODpairId,String timeBeanId,double counter,LinkedHashMap<String,Double> oparams, LinkedHashMap<String, Double> anaParams){
	
	AnalyticalModelODpair odpair=this.odPairs.getODpairset().get(ODpairId);
	String subPopulation = odpair.getSubPopulation();
	
	
	List<AnalyticalModelRoute> routes=odpair.getRoutes();
	Map<Id<AnalyticalModelRoute>,Double> routeFlows=new HashMap<>();
	HashMap<Id<Link>,Double> linkFlows=new HashMap<>();
	
	LinkedHashMap<String,Double> params = this.handleBasicParams(oparams, subPopulation, this.scenario.getConfig());
	
	//double totalUtility=0;
	
	//Calculating route utility for all car routes inside one OD pair.
	
	//HashMap<Id<AnalyticalModelRoute>,Double> oldUtility=new HashMap<>();
	Map<Id<AnalyticalModelRoute>,Double> utility=this.routeUtilities.get(timeBeanId);
	double maxUtil = Double.MIN_VALUE;
	double denominator = 0;
	double expectedMaxUtil = 0;
	for(AnalyticalModelRoute r:routes){
		double u=0;
		
		u=r.calcRouteUtility(params, anaParams,this.networks.get(timeBeanId),this.timeBeans.get(timeBeanId));
		u=u+Math.log(odpair.getAutoPathSize().get(r.getRouteId()));//adding the path size term
		if(maxUtil<u)maxUtil = u;
		utility.put(r.getRouteId(), u);
		expectedMaxUtil+=Math.exp(u*anaParams.get(CNLSUEModel.LinkMiuName));
		//odpair.updateRouteUtility(r.getRouteId(), u,timeBeanId);// Not sure if this is needed
		
		//This Check is to make sure the exp(utility) do not go to infinity.
		if(u>300||u<-300) {
			logger.error("utility is either too small or too large. Increase or decrease the link miu accordingly. The utility is "+u+" for route "+r.getRouteId());
			//throw new IllegalArgumentException("stop!!!");
		}
	}
	this.expectedMaximumCarUtility.get(timeBeanId).put(ODpairId, 1/anaParams.get(CNLSUEModel.LinkMiuName)*Math.log(expectedMaxUtil));
	for(AnalyticalModelRoute r:routes){
		denominator += Math.exp(anaParams.get(CNLSUEModel.LinkMiuName)*(utility.get(r.getRouteId())-maxUtil));
	}
	
	//This is the route flow split
	
	for(AnalyticalModelRoute r:routes){
		double u=utility.get(r.getRouteId());
		double demand = this.carDemand.get(timeBeanId).get(ODpairId);
		double prob = Math.exp(anaParams.get(CNLSUEModel.LinkMiuName)*(u-maxUtil))/denominator;
		this.routeProb.get(timeBeanId).put(r.getRouteId(), prob);
		double flow=prob*demand;
		//For testing purpose, can be removed later
		if(flow==Double.NaN||flow==Double.POSITIVE_INFINITY) {
			logger.error("The flow is NAN. This can happen for a number of reasons. Mostly is total utility of all the routes in a OD pair is zero");
			throw new IllegalArgumentException("Wait!!!!Error!!!!");
		}
		routeFlows.put(r.getRouteId(),flow);
		odpair.getRouteFlow().get(timeBeanId).put(r.getRouteId(), flow);
		this.routeFlow.get(timeBeanId).put(r.getRouteId(), flow);
	}
	for(Id<Link> linkId:this.odPairs.getODpairset().get(ODpairId).getLinkIncidence().keySet()){
		double linkflow=0;
		for(AnalyticalModelRoute r:this.odPairs.getODpairset().get(ODpairId).getLinkIncidence().get(linkId)){
			linkflow+=routeFlows.get(r.getRouteId());
		}
		linkFlows.put(linkId,linkflow);
	}
//	if(this.consecutiveSUEErrorIncrease.get(timeBeanId)>=3) {
//		throw new IllegalArgumentException("Errors are worsenning...!!!");
//	}
	return linkFlows;
}


/**
 * This method does transit sue assignment on the transit network on (Total demand-Car Demand)
 * @param ODpairId
 * @param timeBeanId
 * @param anaParams 
 * @return
 */
protected HashMap<Id<TransitLink>,Double> NetworkLoadingTransitSingleOD(Id<AnalyticalModelODpair> ODpairId,String timeBeanId,int counter,LinkedHashMap<String,Double> oparams, LinkedHashMap<String, Double> anaParams){
	
	AnalyticalModelODpair odpair=this.odPairs.getODpairset().get(ODpairId);
	List<AnalyticalModelTransitRoute> routes=odpair.getTrRoutes(timeBeanId);
//	if(odpair.getODpairId().toString().equals("227014.0_624041.0_person_TCSwithoutCar")) {
//		logger.debug("debug here.");
//	}
	HashMap<Id<AnalyticalModelTransitRoute>,Double> routeFlows=new HashMap<>();
	HashMap<Id<TransitLink>,Double> linkFlows=new HashMap<>();
	String subPopulation = odpair.getSubPopulation();
	LinkedHashMap<String,Double> params = this.handleBasicParams(oparams, subPopulation, this.scenario.getConfig());
	Map<Id<AnalyticalModelTransitRoute>,Double> utility=this.trRouteUtilities.get(timeBeanId);
	double expectedMaxUtil = 0;
	if(routes!=null && routes.size()!=0) {
		double maxUtil = Double.MIN_VALUE;
		double denominator = 0;
		for(AnalyticalModelTransitRoute r:routes){
			double u=0;

			u=r.calcRouteUtility(params, anaParams,
					this.networks.get(timeBeanId),this.transitLinks.get(timeBeanId),this.fareCalculator,null,this.timeBeans.get(timeBeanId));
			u+=Math.log(odpair.getTrPathSize().get(timeBeanId).get(r.getTrRouteId()));//adding the path size term
			if(maxUtil<u)maxUtil = u;
			expectedMaxUtil+=Math.exp(u*anaParams.get(CNLSUEModel.LinkMiuName));
			if(u==Double.NaN) {
				logger.error("The flow is NAN. This can happen for a number of reasons. Mostly is total utility of all the routes in a OD pair is zero");
				throw new IllegalArgumentException("Utility is NAN!!!");
			}

//			if(u>300) {
//				logger.warn("STOP!!!Utility is too large >300");
//			}
			//odpair.updateTrRouteUtility(r.getTrRouteId(), u,timeBeanId);
			String[] part = r.getTrRouteId().toString().split("_");
			String odId = part[0]+"_"+part[1]+"_"+part[2]+"_"+part[3];
			if(!odpair.getODpairId().toString().equals(odId)) {
				logger.debug("Route do not belong to odpair");
			}
			if(utility.containsKey(r.getTrRouteId())) {
				logger.debug("duplicate route key!!!!");
//				String[] part = r.getTrRouteId().toString().split("_");
//				String odId = part[0]+"_"+part[1]+"_"+part[2]+"_"+part[3];
//				if(!odpair.getODpairId().toString().equals(odId))
			}
			utility.put(r.getTrRouteId(), u);

		}
		this.expectedMaximumTrUtility.get(timeBeanId).put(ODpairId, 1/anaParams.get(CNLSUEModel.LinkMiuName)*Math.log(expectedMaxUtil));
		for(AnalyticalModelTransitRoute r:routes){
			denominator += Math.exp(anaParams.get(CNLSUEModel.LinkMiuName)*(utility.get(r.getTrRouteId())-maxUtil));
		}

		for(AnalyticalModelTransitRoute r:routes){
			double totalDemand=this.Demand.get(timeBeanId).get(ODpairId);
			double carDemand=this.carDemand.get(timeBeanId).get(ODpairId);
			double q=(totalDemand-carDemand);
			double u=utility.get(r.getTrRouteId());
			double prob = Math.exp(anaParams.get(CNLSUEModel.LinkMiuName)*(u-maxUtil))/denominator;
			this.trRouteProb.get(timeBeanId).put(r.getTrRouteId(), prob);
			double flow=q*prob;
			if(Double.isNaN(flow)||flow==Double.POSITIVE_INFINITY||flow==Double.NEGATIVE_INFINITY) {
				logger.error("The flow is NAN. This can happen for a number of reasons. Mostly is total utility of all the routes in a OD pair is zero");
				throw new IllegalArgumentException("Error!!!!");
			}
			routeFlows.put(r.getTrRouteId(),flow);
			odpair.getTrRouteFlow().get(timeBeanId).put(r.getTrRouteId(), flow);	
			
			this.trRouteFlow.get(timeBeanId).put(r.getTrRouteId(), flow);
		}

	}
	
	Set<Id<TransitLink>>linksets=this.odPairs.getODpairset().get(ODpairId).getTrLinkIncidence().keySet();
	for(Id<TransitLink> linkId:linksets){
		if(this.transitLinks.get(timeBeanId).containsKey(linkId)) {
		double linkflow=0;
		List<AnalyticalModelTransitRoute>incidence=this.odPairs.getODpairset().get(ODpairId).getTrLinkIncidence().get(linkId);
		for(AnalyticalModelTransitRoute r:incidence){
			List<AnalyticalModelTransitRoute> routesFromOd=routes;
			
			if(routeContain(routesFromOd, r)) {
				linkflow+=routeFlows.get(r.getTrRouteId());
			}
			if(Double.isNaN(linkflow)) {
				logger.error("The flow is NAN. This can happen for a number of reasons. Mostly is total utility of all the routes in a OD pair is zero");
				throw new IllegalArgumentException("Stop!!!");
			}
		}
		linkFlows.put(linkId,linkflow);
		}
	}
	return linkFlows;
}

public static boolean routeContain(List<AnalyticalModelTransitRoute> routesFromOd,AnalyticalModelTransitRoute route) {
	
	for(AnalyticalModelTransitRoute r:routesFromOd) {
		if(r.getTrRouteId().equals(route.getTrRouteId())) {
			route=r;
			return true;
		}
	}
	return false;
}

/**
 * This method should do the network loading for car
 * @param anaParams 
 * @return
 */
protected Map<Id<Link>,Double> performCarNetworkLoading(String timeBeanId, double counter,LinkedHashMap<String,Double> params, LinkedHashMap<String, Double> anaParams){
	Map<Id<Link>,Double> linkVolume=new HashMap<>();
	boolean multiThreading =true;
	if(multiThreading==true) {
		List<Map<Id<Link>, Double>> linkVolumes=Collections.synchronizedList(new ArrayList<>());
		
		this.odPairs.getODpairset().values().parallelStream().forEach(odpair->{
			if(odpair.getRoutes()!=null && this.carDemand.get(timeBeanId).get(odpair.getODpairId())!=0) {
				linkVolumes.add(this.NetworkLoadingCarSingleOD(odpair.getODpairId(),timeBeanId,counter,params,anaParams));
			}
		});
		
		for(Map<Id<Link>,Double>lv:linkVolumes) {
			for(Entry<Id<Link>, Double> d:lv.entrySet()) {
				if(linkVolume.containsKey(d.getKey())) {
					linkVolume.put(d.getKey(), linkVolume.get(d.getKey())+d.getValue());
				}else {
					linkVolume.put(d.getKey(), d.getValue());
				}
			}
		}

	}else {
		for(AnalyticalModelODpair e:this.odPairs.getODpairset().values()){
			if(e.getRoutes()!=null && this.carDemand.get(timeBeanId).get(e.getODpairId())!=0) {
				HashMap <Id<Link>,Double> ODvolume=this.NetworkLoadingCarSingleOD(e.getODpairId(),timeBeanId,counter,params,anaParams);
				for(Id<Link>linkId:ODvolume.keySet()){
					if(linkVolume.containsKey(linkId)){
						linkVolume.put(linkId, linkVolume.get(linkId)+ODvolume.get(linkId));
					}else{
						linkVolume.put(linkId, ODvolume.get(linkId));
					}
				}
			}
		}
	}
	return linkVolume;
}



/**
 * This method should do the network loading for transit
 * @param params 
 * @param anaParams 
 * @return
 */
protected Map<Id<TransitLink>,Double> performTransitNetworkLoading(String timeBeanId,int counter, LinkedHashMap<String, Double> params, LinkedHashMap<String, Double> anaParams){
	Map<Id<TransitLink>,Double> linkVolume=new ConcurrentHashMap<>();
	boolean multiThreading =true;
	if(multiThreading==true) {
		
		List<Map<Id<TransitLink>, Double>> linkTransitVolumes=Collections.synchronizedList(new ArrayList<>());
		
		this.odPairs.getODpairset().values().parallelStream().forEach(odpair->{
//			if(odpair.getODpairId().toString().equals("227014.0_624041.0_person_TCSwithoutCar")) {
//				logger.debug("debug here.");
//			}
			double totalDemand=this.Demand.get(timeBeanId).get(odpair.getODpairId());
			double carDemand=this.carDemand.get(timeBeanId).get(odpair.getODpairId());
			if((totalDemand-carDemand)!=0) {
				linkTransitVolumes.add(this.NetworkLoadingTransitSingleOD(odpair.getODpairId(),timeBeanId,counter,params,anaParams));
			}
//			if((totalDemand-carDemand)==0 && odpair.getTrRoutes(timeBeanId)!=null) {
//				logger.debug("dimension mismatch");
//			}
		});	
		
		for(Map<Id<TransitLink>, Double> lv:linkTransitVolumes) {
			for(Entry<Id<TransitLink>, Double> d:lv.entrySet()) {
				if(linkVolume.containsKey(d.getKey())) {
					linkVolume.put(d.getKey(), linkVolume.get(d.getKey())+d.getValue());
				}else {
					linkVolume.put(d.getKey(), d.getValue());
				}
			}
		}
	}else {

		for(AnalyticalModelODpair e:this.odPairs.getODpairset().values()){
			//this.odPairs.getODpairset().values().parallelStream().forEach((e)->{
			double totalDemand=this.Demand.get(timeBeanId).get(e.getODpairId());
			double carDemand=this.carDemand.get(timeBeanId).get(e.getODpairId());
			if((totalDemand-carDemand)!=0) {
				HashMap <Id<TransitLink>,Double> ODvolume=this.NetworkLoadingTransitSingleOD(e.getODpairId(),timeBeanId,counter,params,anaParams);
				for(Id<TransitLink> linkId:ODvolume.keySet()){
					if(linkVolume.containsKey(linkId)){
						linkVolume.put(linkId, linkVolume.get(linkId)+ODvolume.get(linkId));
					}else{
						linkVolume.put(linkId, ODvolume.get(linkId));
					}
				}
			}
		}
	}
	//});
	//System.out.println(linkVolume.size());
	return linkVolume;
}

/**
 * This method updates the linkCarVolume and linkTransitVolume obtained using MSA 
 * @param linkVolume - Calculated link volume
 * @param transitlinkVolume - Calculated transit volume
 * @param counter - current counter in MSA loop
 * @param timeBeanId - the specific time Bean Id for which the SUE is performed
 */

protected void UpdateLinkVolume(Map<Id<Link>,Double> linkVolume,Map<Id<TransitLink>,Double> transitlinkVolume,int counter,String timeBeanId){
	for(Id<Link> linkId:linkVolume.keySet()) {
		double counterPart=1/beta.get(timeBeanId).get(counter-1);
		//counterPart=1./counter;
		double update=counterPart*this.linkVolumeUpdate.get(timeBeanId).get(linkId);
		((AnalyticalModelLink) this.networks.get(timeBeanId).getLinks().get(linkId)).addLinkCarVolume(update);
	}
	for(Id<TransitLink> trlinkId:transitlinkVolume.keySet()){
		//counterPart=1./counter;
		double counterPart=1/beta.get(timeBeanId).get(counter-1);
		double update=counterPart*this.linkTrVolumeUpdate.get(timeBeanId).get(trlinkId);
		this.transitLinks.get(timeBeanId).get(trlinkId).addPassanger(update,this.networks.get(timeBeanId));
	}
}

/**
 * This method will check for the convergence and also create the error term required for MSA
 * @param linkVolume
 * @param tollerance
 * @return
 */
protected boolean CheckConvergence(Map<Id<Link>,Double> linkVolume,Map<Id<TransitLink>,Double> transitlinkVolume, double tollerance,String timeBeanId,int counter){
	double linkAbove1=0;
	double squareSum=0;
	double sum=0;
	double error=0;
	this.linkVolumeUpdate.put(timeBeanId, new HashMap<>());
	this.linkTrVolumeUpdate.put(timeBeanId, new HashMap<>());
	for(Id<Link> linkid:linkVolume.keySet()){
//		if(linkVolume.get(linkid)==0) {
//			error=0;
//		}else {
			double currentVolume=((AnalyticalModelLink) this.networks.get(timeBeanId).getLinks().get(linkid)).getLinkCarVolume();
			double newVolume=linkVolume.get(linkid);
			this.linkVolumeUpdate.get(timeBeanId).put(linkid, newVolume - currentVolume);
			error=Math.pow((currentVolume-newVolume),2);
			if(error==Double.POSITIVE_INFINITY||error==Double.NEGATIVE_INFINITY) {
				throw new IllegalArgumentException("Error is infinity!!!");
			}
			if(newVolume != 0 && error/(newVolume+.00000001)*100>tollerance) {					
				sum+=1;
			}
			if(error>1) {
				linkAbove1++;
			}
		//}
		
		squareSum+=error;
		if(squareSum==Double.POSITIVE_INFINITY||squareSum==Double.NEGATIVE_INFINITY) {
			throw new IllegalArgumentException("error is infinity!!!");
		}
	}
	for(Id<TransitLink> transitlinkid:transitlinkVolume.keySet()){
//		if(transitlinkVolume.get(transitlinkid)==0) {
//			error=0;
//		}else {
			double currentVolume=this.transitLinks.get(timeBeanId).get(transitlinkid).getPassangerCount();
			double newVolume=transitlinkVolume.get(transitlinkid);
			this.linkTrVolumeUpdate.get(timeBeanId).put(transitlinkid, newVolume-currentVolume);
			error=Math.pow((currentVolume-newVolume),2);
			if(newVolume!=0 && error/(newVolume+.0000001)*100>tollerance) {

				sum+=1;
			}
			if(error>1) {
				linkAbove1++;
			}
		//}
		if(error==Double.NaN||error==Double.NEGATIVE_INFINITY) {
			throw new IllegalArgumentException("Stop!!! There is something wrong!!!");
		}
		squareSum+=error;
	}
	if(squareSum==Double.NaN) {
		System.out.println("WAIT!!!!Problem!!!!!");
	}
	squareSum=Math.sqrt(squareSum);
	if(counter==1) {String id=null;
		this.error.get(timeBeanId).clear();
	}
	this.error.get(timeBeanId).add(squareSum);
	logger.info("ERROR amount for "+timeBeanId+" at sue iteration "+counter+" = "+squareSum);
	//System.out.println("in timeBean Id "+timeBeanId+" No of link not converged = "+sum);
	
//	try {
//		//CNLSUEModel.writeData(timeBeanId+","+counter+","+squareSum+","+sum, this.fileLoc+"ErrorData"+timeBeanId+".csv");
//	} catch (IOException e) {
//		// TODO Auto-generated catch block
//		e.printStackTrace();
//	}
	
	if(counter==1) {
		this.beta.get(timeBeanId).clear();
		//this.error.clear();
		this.beta.get(timeBeanId).add(1.);
	}else {
		if(this.error.get(timeBeanId).get(counter-1)<this.error.get(timeBeanId).get(counter-2)) {
			beta.get(timeBeanId).add(beta.get(timeBeanId).get(counter-2)+this.gammaMSA);
		}else {
			this.consecutiveSUEErrorIncrease.put(timeBeanId, this.consecutiveSUEErrorIncrease.get(timeBeanId)+1);
			beta.get(timeBeanId).add(beta.get(timeBeanId).get(counter-2)+this.alphaMSA);
			
		}
	}
	
	if (squareSum<=1||sum==0||linkAbove1==0){
		return true;
	}else{
		return false;
	}
	
	
	
}
/**
 * This method perform modal Split
 * @param params
 * @param anaParams
 * @param timeBeanId
 */
protected void performModalSplit(LinkedHashMap<String,Double>params,LinkedHashMap<String,Double>anaParams,String timeBeanId) {
	double modeMiu=anaParams.get(CNLSUEModel.ModeMiuName);
	for(AnalyticalModelODpair odPair:this.odPairs.getODpairset().values()){
		//For GV car proportion is always 1
		if(odPair.getSubPopulation()!=null && odPair.getSubPopulation().contains("GV")) {
			double carDemand=this.Demand.get(timeBeanId).get(odPair.getODpairId());
			this.carDemand.get(timeBeanId).put(odPair.getODpairId(),carDemand);
			this.carProbability.get(timeBeanId).put(odPair.getODpairId(), 1.);
			continue;
		// if a phantom trip, car and pt proportion is decided from the simulation and will not be changed
		}else if(odPair.getSubPopulation()!=null && odPair.getSubPopulation().contains("trip")) {
			double carDemand=this.Demand.get(timeBeanId).get(odPair.getODpairId())*odPair.getCarModalSplit();
			this.carDemand.get(timeBeanId).put(odPair.getODpairId(),carDemand);
			this.carProbability.get(timeBeanId).put(odPair.getODpairId(), odPair.getCarModalSplit());
			continue;
		}
		double demand=this.Demand.get(timeBeanId).get(odPair.getODpairId());
		if(demand!=0) { 
			
		Double carUtility=this.expectedMaximumCarUtility.get(timeBeanId).get(odPair.getODpairId());
		Double transitUtility=this.expectedMaximumTrUtility.get(timeBeanId).get(odPair.getODpairId());
		if(carUtility==null)carUtility = Double.NEGATIVE_INFINITY;
		if(transitUtility==null)transitUtility = Double.NEGATIVE_INFINITY;
		if(carUtility==Double.NEGATIVE_INFINITY||transitUtility==Double.POSITIVE_INFINITY||
				Math.exp(transitUtility*modeMiu)==Double.POSITIVE_INFINITY) {
			this.carDemand.get(timeBeanId).put(odPair.getODpairId(), 0.0);
			this.carProbability.get(timeBeanId).put(odPair.getODpairId(), 0.);
			
		}else if(transitUtility==Double.NEGATIVE_INFINITY||carUtility==Double.POSITIVE_INFINITY
				||Math.exp(carUtility*modeMiu)==Double.POSITIVE_INFINITY) {
			this.carDemand.get(timeBeanId).put(odPair.getODpairId(), this.Demand.get(timeBeanId).get(odPair.getODpairId()));
			this.carProbability.get(timeBeanId).put(odPair.getODpairId(), 1.);
		}else if(carUtility==Double.NEGATIVE_INFINITY && transitUtility==Double.NEGATIVE_INFINITY){
			this.carDemand.get(timeBeanId).put(odPair.getODpairId(), 0.);
			this.carProbability.get(timeBeanId).put(odPair.getODpairId(), 0.);
		}else {
			double maxUtil = Math.max(carUtility, transitUtility);
			double carProportion=Math.exp((carUtility-maxUtil)*modeMiu)/(Math.exp((carUtility-maxUtil)*modeMiu)+Math.exp((transitUtility-maxUtil)*modeMiu));
			this.carProbability.get(timeBeanId).put(odPair.getODpairId(), carProportion);
			//System.out.println("Car Proportion = "+carProportion);
			Double cardemand=carProportion*this.Demand.get(timeBeanId).get(odPair.getODpairId());
			if(cardemand==Double.NaN||cardemand==Double.POSITIVE_INFINITY||cardemand==Double.NEGATIVE_INFINITY) {
				logger.error("Car Demand is invalid");
				throw new IllegalArgumentException("car demand is invalid");
			}
			this.carDemand.get(timeBeanId).put(odPair.getODpairId(),cardemand);
		}
	}
	}
}

/**
 * This function will initialize all gradients with zero
 * @param Oparams
 * Be very careful while using the function as it uses all the incidences, which are not ready until the first iteration. The updating can be avoided though.
 * 
 * TODO: Should I use ConcurrentHashMap instead of HashMap?
 * TODO: Should I use parallelStream instead of Stream?
 */
public void initializeGradients(LinkedHashMap<String,Double> Oparams) {
//	Map<String,Double> zeroGrad = new HashMap<>();
	this.gradientKeys = ODUtils.extractODVarKeys(Oparams.keySet());
	this.gradientArray = new MapToArray<String>("Gradient",this.gradientKeys);
//	this.gradientKeys.forEach(k->{
//		zeroGrad.put(k, 0.);
//	});
	this.gradMultiplier = MatrixUtils.createRealVector(new double[this.gradientKeys.size()]).mapAdd(1.).toArray();
	for(String timeId:this.timeBeans.keySet()) {
		this.linkGradient.put(timeId, new HashMap<>());
		//this.linkGradient.get(timeId).putAll(this.networks.get(timeId).getLinks().keySet().parallelStream().collect(Collectors.toMap(kk->kk, kk->new HashMap<>(zeroGrad))));
		this.linkGradient.get(timeId).putAll(this.linkIncidenceMatrix.keySet().parallelStream().collect(Collectors.toMap(kk->kk, kk->new double[this.gradientKeys.size()])));
		
		this.linkTTGradient.put(timeId, new HashMap<>());
		//this.linkTTGradient.get(timeId).putAll(this.networks.get(timeId).getLinks().keySet().parallelStream().collect(Collectors.toMap(kk->kk, kk->new HashMap<>(zeroGrad))));
		this.linkTTGradient.get(timeId).putAll(this.linkIncidenceMatrix.keySet().parallelStream().collect(Collectors.toMap(kk->kk, kk->new double[this.gradientKeys.size()])));
		
		this.trLinkGradient.put(timeId, new HashMap<>());
		//this.trLinkGradient.get(timeId).putAll(this.transitLinks.get(timeId).keySet().parallelStream().collect(Collectors.toMap(kk->kk, kk->new HashMap<>(zeroGrad))));
		this.trLinkGradient.get(timeId).putAll(this.transitLinks.get(timeId).keySet().parallelStream().collect(Collectors.toMap(kk->kk, kk->new double[this.gradientKeys.size()])));
		
		this.trLinkTTGradient.put(timeId, new HashMap<>());
		//this.trLinkTTGradient.get(timeId).putAll(this.transitLinks.get(timeId).keySet().parallelStream().collect(Collectors.toMap(kk->kk, kk->new HashMap<>(zeroGrad))));
		this.trLinkTTGradient.get(timeId).putAll(this.transitLinks.get(timeId).keySet().parallelStream().collect(Collectors.toMap(kk->kk, kk->new double[this.gradientKeys.size()])));
		
		this.routeFlowGradient.put(timeId, new HashMap<>());
		//this.routeFlowGradient.get(timeId).putAll(this.routeProb.get(timeId).keySet().parallelStream().collect(Collectors.toMap(kk->kk, kk->new HashMap<>(zeroGrad))));
		this.routeFlowGradient.get(timeId).putAll(this.routeProb.get(timeId).keySet().parallelStream().collect(Collectors.toMap(kk->kk, kk->new double[this.gradientKeys.size()])));
		
		this.trRouteFlowGradient.put(timeId, new HashMap<>());
		//this.trRouteFlowGradient.get(timeId).putAll(this.trRouteProb.get(timeId).keySet().parallelStream().collect(Collectors.toMap(kk->kk, kk->new HashMap<>(zeroGrad))));
		this.trRouteFlowGradient.get(timeId).putAll(this.trRouteProb.get(timeId).keySet().parallelStream().collect(Collectors.toMap(kk->kk, kk->new double[this.gradientKeys.size()])));
		
		
		this.fareLinkGradient.put(timeId, new HashMap<>());
		//this.fareLinkGradient.get(timeId).putAll(this.fareLinkincidenceMatrix.get(timeId).keySet().parallelStream().collect(Collectors.toMap(kk->kk, kk->new HashMap<>(zeroGrad))));
		this.fareLinkGradient.get(timeId).putAll(this.fareLinkincidenceMatrix.get(timeId).keySet().parallelStream().collect(Collectors.toMap(kk->kk, kk->new double[this.gradientKeys.size()])));
		
		this.odParameterIncidence.put(timeId, new HashMap<>());
		this.odParameterIncidence.get(timeId).putAll(this.odPairs.getODpairset().keySet().parallelStream().collect(Collectors.toMap(kk->kk, kk->new double[this.gradientKeys.size()])));
	}
	this.intiializeGradient = false;
	this.linkGradL1NormThreshold = this.gradientKeys.size()*3600;
	logger.info("Finished initializing gradients");
}

/**
 * This is a huge step forward. This class can calculate gradient using backpropagation
 * Still not checked
 * @param population
 * @param counter
 * @param Oparams
 * @param anaParam
 */
public void caclulateGradient(String timeId, int counter, LinkedHashMap<String,Double> oparams, LinkedHashMap<String,Double>anaParam) {
	this.caclulateGradient(timeId, counter, oparams, anaParam, false);
}

/**
 * This is a huge step forward. This class can calculate gradient using backpropagation
 * Still not checked
 * @param population
 * @param counter
 * @param Oparams
 * @param anaParam
 */
public void caclulateGradient(String timeId, int counter, LinkedHashMap<String,Double> oparams, LinkedHashMap<String,Double>anaParam, boolean useUnitUpdateWeight) {
	//RealVector p = null;
	
	if(this.intiializeGradient) {//maybe its better to do it once in the generate od pair and then not do it again 
		this.initializeGradients(oparams);
		//System.out.println("GB: " + (double) (Runtime.getRuntime().totalMemory() - Runtime.getRuntime().freeMemory()) / (1024*1024*1024));
		
	}else {
		//Calculate the travel time gradients
		this.linkTTGradient.get(timeId).entrySet().parallelStream().forEach(linkGradientMap-> {
			//for(Entry<Id<Link>,Map<String,Double>> linkGradientMap:timeMap.getValue().entrySet()) {
			CNLLink link = (CNLLink) this.networks.get(timeId).getLinks().get(linkGradientMap.getKey());
			//for(Entry<String, Double> var:linkGradientMap.getValue().entrySet()) {
				double flow = link.getLinkCarVolume()+link.getLinkTransitVolume();
				double t_0 = link.getLength()/link.getFreespeed();//should be in sec
				double cap = link.getCapacity()*(this.timeBeans.get(timeId).getSecond()-this.timeBeans.get(timeId).getFirst())/3600;
				double beta = anaParam.get(ODDifferentiableSUEModel.BPRbetaName);
				double alpha = anaParam.get(ODDifferentiableSUEModel.BPRalphaName);
				double cons = alpha*beta*t_0/Math.pow(cap, beta)*Math.pow(flow,beta-1)/3600;
				//double grad = anaParam.get(ODDifferentiableSUEModel.BPRalphaName)*beta*t_0/Math.pow(cap, beta)*Math.pow(flow,beta-1)*this.linkGradient.get(timeId).get(link.getId()).get(var.getKey());
				if(cons>3600) {
					logger.debug("timeGradient is too high");
				}
				double[] g = MatrixUtils.createRealVector(this.linkGradient.get(timeId).get(link.getId())).mapMultiply(cons).toArray();
				//this.linkTTGradient.get(timeId).get(link.getId()).put(var.getKey(),grad);
				this.linkTTGradient.get(timeId).put(link.getId(),g);
				
			//}
			
		});
		
		this.trLinkTTGradient.get(timeId).entrySet().parallelStream().forEach(linkGradientMap->{	
			TransitLink link = this.transitLinks.get(timeId).get(linkGradientMap.getKey());
			//for(Entry<String, Double> var:linkGradientMap.getValue().entrySet()) {
				if(link instanceof TransitDirectLink) {
					CNLTransitDirectLink dlink = (CNLTransitDirectLink)link;
					double grad = 0;
					RealVector g = MatrixUtils.createRealVector(new double[this.gradientKeys.size()]);
					for(Id<Link> linkId:dlink.getLinkList()) {
						if(this.linkTTGradient.get(timeId).get(linkId)==null) {//As we have used the link plan incidence to loop, there might be some link not used by any od pairs.
							//For these links, no matter what the decision variables are, the flow will not change (flows are from transit vehicle flow only). So, for these links, we can assume the gradient
							//to be zero.// Not sure if needed here
							logger.debug("Dead link here. Putting gradient = 0.");
							
						}else {
							//grad+=this.linkTTGradient.get(timeId).get(linkId).get(var.getKey());
							g = g.add(MatrixUtils.createRealVector(this.linkTTGradient.get(timeId).get(linkId)));
						}	
					}
					if(Double.isNaN(grad))
						logger.debug("Debug point. Gradient is NAN");
					//this.trLinkTTGradient.get(timeId).get(dlink.getTrLinkId()).put(var.getKey(), grad);
					this.trLinkTTGradient.get(timeId).put(dlink.getTrLinkId(),g.toArray());
				}else if(link instanceof TransitTransferLink){
					CNLTransitTransferLink transferLink = (CNLTransitTransferLink)link;
					CNLTransitDirectLink dlink = transferLink.getNextdLink();
					double grad = 0;
					double[] g = new double[this.gradientKeys.size()];
					if(dlink != null) {//For an alighting link only (the last transfer leg) the next dlink is null. 
						//The gradient for this link's travel time is zero as the waiting time for a alighting only link is always zero.
						CNLLink plink = (CNLLink) this.networks.get(timeId).getLinks().get(transferLink.getStartingLinkId());
						double headway = dlink.getHeadway();
						double cap = dlink.getCapacity();
						double freq = dlink.getFrequency();
						double beta = anaParam.get(ODDifferentiableSUEModel.TransferbetaName);
						//double passengerTobeBorded = transferLink.getPassangerCount();
						double passengerOnBord = plink.getTransitPassengerVolume(dlink.getLineId()+"_"+dlink.getRouteId());
						double volume = passengerOnBord;
						double grad1 = beta*headway/Math.pow(cap*freq, beta)*Math.pow(volume, beta-1)/3600;//if both the second and first term is 
						if(grad1>3600) {
							logger.debug("waiting time gradient is too high!");
						}
						if(Double.isInfinite(grad1)||Double.isNaN(grad1))grad1 = 0;
						//double grad2 = this.trLinkGradient.get(timeId).get(transferLink.getTrLinkId()).get(var.getKey());
						//RealVector g2 = MatrixUtils.createRealVector(this.trLinkGradient.get(timeId).get(transferLink.getTrLinkId()));//Why this?
						RealVector g2 = MatrixUtils.createRealVector(new double[this.gradientKeys.size()]);
						for(Id<TransitLink> l:transferLink.getIncidentLinkIds()){
							//grad2+=this.trLinkGradient.get(timeId).get(l).get(var.getKey());
							g2 = g2.add(MatrixUtils.createRealVector(this.trLinkGradient.get(timeId).get(l)));
						}
						//grad = grad1*grad2;
						g = g2.mapMultiply(grad1).toArray();
						
					}else {
						grad = 0;
					}
					if(Double.isNaN(grad))
						logger.debug("Debug point. Gradient is NAN");
					//this.trLinkTTGradient.get(timeId).get(transferLink.getTrLinkId()).put(var.getKey(),grad);
					this.trLinkTTGradient.get(timeId).put(transferLink.getTrLinkId(),g);
				}
			//}
		});
		
	}
	
	
	this.odPairs.getODpairset().entrySet().parallelStream().forEach(od->{
//		for(String var:this.gradientKeys) {
			//calculate the auto route utility gradient first
			
			LinkedHashMap<String,Double> params = this.handleBasicParams(oparams, od.getValue().getSubPopulation(), this.scenario.getConfig());
//			Map<Id<AnalyticalModelRoute>,Double> routeUGradient = new HashMap<>();
			Map<Id<AnalyticalModelRoute>,double[]> routeUGrad = new HashMap<>();
			
//			double carUGrad = 0;
			RealVector carUGradient = MatrixUtils.createRealVector(new double[this.gradientKeys.size()]);
			if( od.getValue().getRoutes()!=null) {
			for(AnalyticalModelRoute route: od.getValue().getRoutes()) {
//				double uGradient = 0;
				RealVector uGrad = MatrixUtils.createRealVector(new double[this.gradientKeys.size()]);
				for(Id<Link>linkId: route.getLinkIds()) {
//					uGradient+=this.linkTTGradient.get(timeId).get(linkId).get(var);
					uGrad = uGrad.add(MatrixUtils.createRealVector(this.linkTTGradient.get(timeId).get(linkId)));
				}
				//uGradient*=(params.get(CNLSUEModel.MarginalUtilityofTravelCarName)-params.get(CNLSUEModel.MarginalUtilityofPerformName));
				uGrad.mapMultiplyToSelf((params.get(CNLSUEModel.MarginalUtilityofTravelCarName)-params.get(CNLSUEModel.MarginalUtilityofPerformName)));
				//if(Double.isNaN(uGradient))logger.debug("Debug point. Gradient is NAN");
//				routeUGradient.put(route.getRouteId(), uGradient);
				routeUGrad.put(route.getRouteId(), uGrad.toArray());
//				carUGrad+=this.routeProb.get(timeId).get(route.getRouteId())*uGradient;
				carUGradient = carUGradient.add(uGrad.mapMultiply(this.routeProb.get(timeId).get(route.getRouteId())));
			}
			if(od.getValue().getRoutes().size()!=routeUGrad.size()) {
				logger.debug("route size mismatch!!!");
			}
			}
//			Map<Id<AnalyticalModelTransitRoute>,Double> trRouteUGradient = new HashMap<>();	
			Map<Id<AnalyticalModelTransitRoute>,double[]> trRouteUGrad = new HashMap<>();
//			double trUGradient = 0;
			RealVector trUtGrad = MatrixUtils.createRealVector(new double[this.gradientKeys.size()]);
			if(od.getValue().getTrRoutes(timeId)!=null) {
			for(AnalyticalModelTransitRoute trRoute : od.getValue().getTrRoutes(timeId)) {
				
//				double routeGradientDlink = 0;
				RealVector routeGradDlink = MatrixUtils.createRealVector(new double[this.gradientKeys.size()]);
//				double routeGradientTRLink = 0;
				RealVector routeGradTRLink = MatrixUtils.createRealVector(new double[this.gradientKeys.size()]);
				for(TransitDirectLink dlink:trRoute.getTransitDirectLinks()) {
//					routeGradientDlink += this.trLinkTTGradient.get(timeId).get(dlink.getTrLinkId()).get(var);
					routeGradDlink = routeGradDlink.add(MatrixUtils.createRealVector(this.trLinkTTGradient.get(timeId).get(dlink.getTrLinkId())));
				}
//				routeGradientDlink*=params.get(CNLSUEModel.MarginalUtilityofTravelptName)-params.get(CNLSUEModel.MarginalUtilityofPerformName);
				routeGradDlink.mapMultiplyToSelf(params.get(CNLSUEModel.MarginalUtilityofTravelptName)-params.get(CNLSUEModel.MarginalUtilityofPerformName));
				for(TransitTransferLink trlink:trRoute.getTransitTransferLinks()) {
//					routeGradientTRLink += this.trLinkTTGradient.get(timeId).get(trlink.getTrLinkId()).get(var);
					routeGradTRLink = routeGradTRLink.add(MatrixUtils.createRealVector(this.trLinkTTGradient.get(timeId).get(trlink.getTrLinkId())));
				}
//				routeGradientTRLink*=params.get(CNLSUEModel.MarginalUtilityofWaitingName)-params.get(CNLSUEModel.MarginalUtilityofPerformName);
				routeGradTRLink.mapMultiplyToSelf(params.get(CNLSUEModel.MarginalUtilityofWaitingName)-params.get(CNLSUEModel.MarginalUtilityofPerformName));
//				double grad = routeGradientDlink+routeGradientTRLink;
				RealVector g = routeGradDlink.add(routeGradTRLink);
				
//				if(Double.isNaN(grad))
//					logger.debug("Debug point. Gradient is NAN");
//				trRouteUGradient.put(trRoute.getTrRouteId(), grad);
				trRouteUGrad.put(trRoute.getTrRouteId(), g.toArray());
				
//				trUGradient += this.trRouteProb.get(timeId).get(trRoute.getTrRouteId())*grad;
				trUtGrad = trUtGrad.add(g.mapMultiply(this.trRouteProb.get(timeId).get(trRoute.getTrRouteId())));
			}
			}
//			if(od.getValue().getSubPopulation()!=null && (od.getValue().getSubPopulation().contains("GV")||od.getValue().getSubPopulation().contains("trip"))) {
//				//carUGrad = 0;
//				carUGradient.mapMultiplyToSelf(0);
//				//trUGradient=0;
//				trUtGrad.mapMultiplyToSelf(0);
//			}
			
			//Calculate route flow gradient 
			double pm = 0;
			if(this.carProbability.get(timeId).get(od.getKey())!=null) pm = this.carProbability.get(timeId).get(od.getKey()); 
//			double modeConst = pm*carUGrad+(1-pm)*trUGradient;
			RealVector modeC = carUGradient.mapMultiply(pm).add(trUtGrad.mapMultiply(1-pm));
			if(modeC.getNorm()>20000) {
				logger.debug("Too High mode const");
			}
			double d = this.Demand.get(timeId).get(od.getKey());
			//Map<String,Double> odInc = new HashMap<>();
//			for(Entry<Id<AnalyticalModelRoute>, Double> rId:routeUGradient.entrySet()) {
			
			
			for(Entry<Id<AnalyticalModelRoute>, double[]> rId:routeUGrad.entrySet()) {
				double pr = this.routeProb.get(timeId).get(rId.getKey());
		//		double term0 = pm*d*pr*anaParam.get(CNLSUEModel.LinkMiuName)*(rId.getValue()-carUGrad);
				RealVector t0 = MatrixUtils.createRealVector(rId.getValue()).subtract(carUGradient).mapMultiply(pm*d*pr*anaParam.get(CNLSUEModel.LinkMiuName));
		//		double term1 = pr*d*pm*anaParam.get(CNLSUEModel.ModeMiuName)*(carUGrad-modeConst);
				RealVector t1 = carUGradient.subtract(modeC).mapMultiply(pr*d*pm*anaParam.get(CNLSUEModel.ModeMiuName));
		//		Map<String,Double> tt2 = new HashMap<>();
				
				if(this.ifODParameterIncidence) {
					Map<String,Double> oi = new HashMap<>();
					for(String var1:this.gradientKeys) {
						double term2 = ODUtils.ifMatch_1_else_0(od.getKey(),this.odPairs.getODpairset().get(od.getKey()).getSubPopulation(), timeId, var1);
						oi.put(var1, term2);
					}
					this.odParameterIncidence.get(timeId).put(od.getKey(), this.gradientArray.getMatrix(oi));
				}
				RealVector odInc = MatrixUtils.createRealVector(this.odParameterIncidence.get(timeId).get(od.getKey()));
				RealVector p = this.gradientArray.getRealVector(oparams);
				RealVector tt2 = odInc.ebeDivide(p).mapMultiply(pr*pm*d).ebeDivide(MatrixUtils.createRealVector(this.gradMultiplier));
//				for(String var1:this.gradientKeys) {
//					//double term2 = pr*pm*ODUtils.ifMatch_1_else_0(od.getKey(),this.odPairs.getODpairset().get(od.getKey()).getSubPopulation(), timeId, var1)*d/params.get(var1);
//					double term2 = pr*pm*ODUtils.ifMatch_1_else_0(od.getKey(),this.odPairs.getODpairset().get(od.getKey()).getSubPopulation(), timeId, var1)*d/params.get(var1);
//					tt2.put(var1, term2);
//				}
//				if(tt2.size()!=this.gradientKeys.size()) {
//					System.out.println();
//				}
//				if(this.gradientArray.getKeySet().size()!=this.gradientKeys.size()) {
//					System.out.println();
//				}
//				double grad = term0 + term1 + term2;
				//RealVector g = t0.add(t1).add(this.gradientArray.getRealVector(tt2));
				RealVector g = t0.add(t1).add(tt2);
//				this.routeFlowGradient.get(timeId).get(rId.getKey()).put(var, grad);
				if(g.isInfinite()||g.isNaN()) {
					logger.error("Nan or infinite gradient!!! Check");
				}
				this.routeFlowGradient.get(timeId).put(rId.getKey(), g.toArray());
			}
			
//			for(Entry<Id<AnalyticalModelTransitRoute>, Double> trUGrad: trRouteUGradient.entrySet()) {
			for(Entry<Id<AnalyticalModelTransitRoute>, double[]> trUGrad: trRouteUGrad.entrySet()) {
				double pr = this.trRouteProb.get(timeId).get(trUGrad.getKey());
//				double term0 = (1-pm)*d*pr*anaParam.get(CNLSUEModel.LinkMiuName)*(trUGrad.getValue()-trUGradient);
				RealVector t0 = trUtGrad.subtract(MatrixUtils.createRealVector(trUGrad.getValue())).mapMultiplyToSelf((1-pm)*d*pr*anaParam.get(CNLSUEModel.LinkMiuName));
//				double term1 = pr*d*(1-pm)*anaParam.get(CNLSUEModel.ModeMiuName)*(trUGradient-modeConst);
				RealVector t1 = trUtGrad.subtract(modeC).mapMultiplyToSelf(pr*d*(1-pm)*anaParam.get(CNLSUEModel.ModeMiuName));
//				double term2 = pr*(1-pm)*ODUtils.ifMatch_1_else_0(od.getKey(),this.odPairs.getODpairset().get(od.getKey()).getSubPopulation(), timeId, var)*d/params.get(var);
//				double grad = term0 + term1 + term2;
				//Map<String,Double> tt2 = new HashMap<>();
//				for(String var1:this.gradientKeys) {
//					//double term2 = pr*pm*ODUtils.ifMatch_1_else_0(od.getKey(),this.odPairs.getODpairset().get(od.getKey()).getSubPopulation(), timeId, var1)*d/params.get(var1);
//					double term2 = pr*(1-pm)*ODUtils.ifMatch_1_else_0(od.getKey(),this.odPairs.getODpairset().get(od.getKey()).getSubPopulation(), timeId, var1)*d/params.get(var1);
//					tt2.put(var1, term2);
//				}
				RealVector odInc = MatrixUtils.createRealVector(this.odParameterIncidence.get(timeId).get(od.getKey()));
				RealVector p = this.gradientArray.getRealVector(oparams);
				RealVector tt2 = odInc.ebeDivide(p).mapMultiply(pr*(1-pm)*d).ebeDivide(MatrixUtils.createRealVector(this.gradMultiplier));
				//RealVector g = t0.add(t1).add(this.gradientArray.getRealVector(tt2));
				RealVector g = t0.add(t1).add(tt2);
//				this.trRouteFlowGradient.get(timeId).get(trUGrad.getKey()).put(var, grad);
				this.trRouteFlowGradient.get(timeId).put(trUGrad.getKey(), g.toArray());
			}
//		}
		//this.applyODBasedGradeintClipping(routeUGrad.keySet(), trRouteUGrad.keySet(), od.getKey(), timeId);
	});
	if(this.ifGradMultiply && (counter == 1||counter%10==0)) {//For a newly started TA cycle, both t0 and t1 terms in transit and car routes are zero. 
		//As a result, that is used to derive the gradient multiplier. The multiplier can be directly applied to the route and tr route gradient as well for the same reason
		// However, at later stage the old gradient is added with the new gradient. So, to make the basis of the two gradient same, the multiplier has to be added at the variable gradient source. 
		//In this model, which is vector tt2 in lines 1344 and 1380. Ashraf, Dec20.
		double[] m = new double[this.gradientKeys.size()];
		for(Entry<String, Map<Id<AnalyticalModelRoute>, double[]>> t:this.routeFlowGradient.entrySet()){
			for(double[]d:t.getValue().values())m =findAbsMax(m,d);
		}
		
		for(Entry<String, Map<Id<AnalyticalModelTransitRoute>, double[]>> t:this.trRouteFlowGradient.entrySet()){
			for(double[]d:t.getValue().values())m =findAbsMax(m,d);
		}
		
		if(counter == 1) {
			for(int i = 0;i<m.length;i++) {
				if(m[i]!=0 && (m[i]>this.maxAbsGrad||m[i]<this.minAbsGrad)) {
					this.gradMultiplier[i] =2*m[i]/(this.maxAbsGrad+this.minAbsGrad);
					if(Double.isInfinite(2*m[i]/(this.maxAbsGrad+this.minAbsGrad))||Double.isNaN(2*m[i]/(this.maxAbsGrad+this.minAbsGrad))||2*m[i]/(this.maxAbsGrad+this.minAbsGrad)==0) {
						logger.debug("multiplier is either infinite, nan or 0!!!");
					}
				}
			}
			this.routeFlowGradient.values().forEach(t->{
				t.values().forEach(d->{
					for(int i = 0;i<d.length;i++)d[i]=d[i]/this.gradMultiplier[i];
				});
			});
			
			this.trRouteFlowGradient.values().forEach(t->{
				t.values().forEach(d->{
					for(int i = 0;i<d.length;i++)d[i]=d[i]/this.gradMultiplier[i];
				});
			});
		}else {
			double norm = 0;
			for(double d:m)norm+=Math.abs(d);
			if(Double.isInfinite(norm)) {
				logger.debug("max grad norm infinity!!!");
			}
			if(norm>this.maxAbsL1Norm) {
				this.scaleBackGradients();
				this.gradMultiplier = MatrixUtils.createRealVector(new double[m.length]).mapAdd(1).mapMultiply(norm/this.maxAbsL1Norm).toArray();
			}
			this.scaleGradients();
		}

	}
	
	
	
	if(this.ifODParameterIncidence)this.ifODParameterIncidence = false;
	//finally the link volume and MaaSPackage usage gradient update
	
	this.linkGradient.get(timeId).entrySet().parallelStream().forEach(linkId->{
//		for(String var:this.gradientKeys) {
//			double oldGrad = this.linkGradient.get(timeId).get(linkId.getKey()).get(var);
			RealVector old = MatrixUtils.createRealVector(this.linkGradient.get(timeId).get(linkId.getKey()));
//			double grad = 0;
			RealVector g = MatrixUtils.createRealVector(new double[this.gradientKeys.size()]);
			for(Id<AnalyticalModelRoute>r:this.linkIncidenceMatrix.get(linkId.getKey())) {
//				grad+=this.routeFlowGradient.get(timeId).get(r).get(var);
				g = g.add(MatrixUtils.createRealVector(this.routeFlowGradient.get(timeId).get(r)));
			}
			//double norm = 0;
			//if((norm = g.getL1Norm())>this.linkGradL1NormThreshold)g = g.mapDivide(norm).mapMultiply(this.linkGradL1NormThreshold);
//			double gradUpdate = oldGrad + (grad-oldGrad)*(1/this.beta.get(timeId).get(counter-1));
			double updateWeight = 1/this.beta.get(timeId).get(counter-1);
			if(useUnitUpdateWeight) updateWeight = 1;
			RealVector gUpdate = old.add(g.subtract(old).mapMultiplyToSelf(updateWeight));
//			this.linkGradient.get(timeId).get(linkId.getKey()).put(var, gradUpdate);
			this.linkGradient.get(timeId).put(linkId.getKey(),gUpdate.toArray());
//		}
	});
//	double totR = 0;
//	Set<Id<AnalyticalModelTransitRoute>> rIds = new HashSet<>();
//	for(AnalyticalModelODpair od:this.odPairs.getODpairset().values()) {
//		if(od.getTrRoutes(timeId)!=null) {
//			totR+=od.getTrRoutes(timeId).size();
//			od.getTrRoutes(timeId).stream().forEach(r->rIds.add(r.getTrRouteId()));
//		}
//	}
//	System.out.println("transit routes = "+totR);
//	System.out.println("unique transit routes = "+ rIds.size());
	
	this.trLinkGradient.get(timeId).entrySet().parallelStream().forEach(linkId->{	
//		for(String var:this.gradientKeys) {
//			double oldGrad = this.trLinkGradient.get(timeId).get(linkId.getKey()).get(var);
			RealVector old = MatrixUtils.createRealVector(this.trLinkGradient.get(timeId).get(linkId.getKey()));
//			double grad = 0;
			RealVector g = MatrixUtils.createRealVector(new double[this.gradientKeys.size()]);
			for(Id<AnalyticalModelTransitRoute>r:this.trLinkIncidenceMatrix.get(timeId).get(linkId.getKey())) {
//				grad+=this.trRouteFlowGradient.get(timeId).get(r).get(var);
				if(this.trRouteFlowGradient.get(timeId).get(r)!=null) {
					g = g.add(MatrixUtils.createRealVector(this.trRouteFlowGradient.get(timeId).get(r)));
				}
			}
			//double norm = 0;
			//if((norm = g.getL1Norm())>this.linkGradL1NormThreshold)g = g.mapDivide(norm).mapMultiply(this.linkGradL1NormThreshold);
//			double gradUpdate = oldGrad + (grad-oldGrad)*(1/this.beta.get(timeId).get(counter-1));
			double updateWeight = 1/this.beta.get(timeId).get(counter-1);
			if(useUnitUpdateWeight) updateWeight = 1;
			RealVector gradU = old.add(g.subtract(old).mapMultiplyToSelf(updateWeight));
			
//			this.trLinkGradient.get(timeId).get(linkId.getKey()).put(var, gradUpdate);
			this.trLinkGradient.get(timeId).put(linkId.getKey(), gradU.toArray());
//		}
	});
	this.fareLinkGradient.get(timeId).entrySet().parallelStream().forEach(linkId->{
//		for(String var:this.gradientKeys) {
//			double grad = 0;
			RealVector g = MatrixUtils.createRealVector(new double[this.gradientKeys.size()]);
			for(Id<AnalyticalModelTransitRoute>r:this.fareLinkincidenceMatrix.get(timeId).get(linkId.getKey())) {
//				grad+=this.trRouteFlowGradient.get(timeId).get(r).get(var);
				if(this.trRouteFlowGradient.get(timeId).get(r)!=null) {
					g = g.add(MatrixUtils.createRealVector(this.trRouteFlowGradient.get(timeId).get(r)));
				}
			}
//			this.fareLinkGradient.get(timeId).get(linkId.getKey()).put(var, grad);
			this.fareLinkGradient.get(timeId).put(linkId.getKey(), g.toArray());
//		}
	});
	logger.info("Finished Calulating Gradient.");
}




/**
 * This is the same method and does the same task as perform SUE, but takes the internal Parameters as an input too.
 * This will be used for the internal parameters calibration internally
 * @param params
 * @return
 */
//@Override
//public SUEModelOutput perFormSUE(LinkedHashMap<String, Double> params,LinkedHashMap<String,Double> anaParams) {
//	this.resetCarDemand();
//	
//	LinkedHashMap<String,Double> inputParams=new LinkedHashMap<>(params);
//	LinkedHashMap<String,Double> inputAnaParams=new LinkedHashMap<>(anaParams);
//	//Loading missing parameters from the default values		
//	Map<String,Map<Id<Link>,Double>> outputLinkFlow=new HashMap<>();
//	
//	Map<String,Map<Id<TransitLink>,Double>> outputTrLinkFlow=new HashMap<>();
//	
//	
//	
//	//Checking and updating for the parameters 
//	for(Entry<String,Double> e:this.Params.entrySet()) {
//		if(!params.containsKey(e.getKey())) {
//			params.put(e.getKey(), e.getValue());
//		}
//	}
//	
//	//Checking and updating for the analytical model parameters
//	for(Entry<String,Double> e:this.AnalyticalModelInternalParams.entrySet()) {
//		if(!anaParams.containsKey(e.getKey())) {
//			anaParams.put(e.getKey(), e.getValue());
//		}
//	}
//	
//	//Creating different threads for different time beans
//	Thread[] threads=new Thread[this.timeBeans.size()];
//	int i=0;
//	for(String timeBeanId:this.timeBeans.keySet()) {
//		threads[i]=new Thread(new SUERunnable(this,timeBeanId,params,anaParams),timeBeanId);
//		i++;
//		outputLinkFlow.put(timeBeanId, new HashMap<Id<Link>, Double>());
//		outputLinkTT.put(timeBeanId, new HashMap<Id<Link>, Double>());
//		outputTrLinkFlow.put(timeBeanId, new HashMap<Id<TransitLink>, Double>());
//		outputTrLinkTT.put(timeBeanId, new HashMap<Id<TransitLink>, Double>());
//	}
//	//Starting the Threads
//	for(i=0;i<this.timeBeans.size();i++) {
//		threads[i].start();
//	}
//	
//	//joining the threads
//	for(i=0;i<this.timeBeans.size();i++) {
//		try {
//			threads[i].join();
//		} catch (InterruptedException e1) {
//			e1.printStackTrace();
//		}
//	}
//	
//	//Collecting the Link Flows
//	for(String timeBeanId:this.timeBeans.keySet()) {
//		for(Id<Link> linkId:this.getNetworks().get(timeBeanId).getLinks().keySet()) {
//			outputLinkFlow.get(timeBeanId).put(linkId, 
//					((AnalyticalModelLink) this.getNetworks().get(timeBeanId).getLinks().get(linkId)).getLinkAADTVolume());
//		}
//	}
//	
//	//Collecting the Link Transit 
//	for(String timeBeanId:this.timeBeans.keySet()) {
//		for(Id<TransitLink> linkId:this.transitLinks.get(timeBeanId).keySet()) {
//			outputTrLinkFlow.get(timeBeanId).put(linkId, 
//					(this.transitLinks.get(timeBeanId).get(linkId).getPassangerCount()));
//		}
//	}
//	
//	//collect pt occupancy
//	Map<String, Map<Id<Link>, Double>> averagePtOccupancyOnLink=new HashMap<>();
//	for(String timeBeanId:this.timeBeans.keySet()) {
//		averagePtOccupancyOnLink.put(timeBeanId, new HashMap<>());
//		for(Id<Link>linkId:this.totalPtCapacityOnLink.get(timeBeanId).keySet()) {
//			double occupancy=((CNLLink)this.networks.get(timeBeanId).getLinks().get(linkId)).getLinkTransitPassenger()/this.totalPtCapacityOnLink.get(timeBeanId).get(linkId);
//			averagePtOccupancyOnLink.get(timeBeanId).put(linkId, occupancy);
//		}
//	}
//	
//	SUEModelOutput out=new SUEModelOutput(outputLinkFlow, outputTrLinkFlow, this.outputLinkTT, this.outputTrLinkTT);
//	out.setAveragePtOccupancyOnLink(averagePtOccupancyOnLink);
//	//new OdInfoWriter("toyScenario/ODInfo/odInfo",this.timeBeans).writeOdInfo(this.getOdPairs(), getDemand(), getCarDemand(), inputParams, inputAnaParams);
//	return out;
//}

private void resetCarDemand() {
	
	
	for(String timeId:this.timeBeans.keySet()) {
		this.carDemand.put(timeId, new HashMap<Id<AnalyticalModelODpair>, Double>());
		this.routeUtilities.put(timeId,new ConcurrentHashMap<>());
		this.trRouteUtilities.put(timeId, new ConcurrentHashMap<>());
		this.routeFlow.put(timeId, new ConcurrentHashMap<>());
		this.trRouteFlow.put(timeId, new ConcurrentHashMap<>());
		this.routeProb.put(timeId, new ConcurrentHashMap<>());
		this.trRouteProb.put(timeId, new ConcurrentHashMap<>());
		this.expectedMaximumCarUtility.put(timeId, new ConcurrentHashMap<>());
		this.expectedMaximumTrUtility.put(timeId, new ConcurrentHashMap<>());
		for(Id<AnalyticalModelODpair> o:this.Demand.get(timeId).keySet()) {
			this.carDemand.get(timeId).put(o, this.Demand.get(timeId).get(o)*0.5);//This needs more attention
			
		}

	}
}

@Deprecated
public Measurements perFormSUE(LinkedHashMap<String, Double> params,LinkedHashMap<String,Double> anaParams,Measurements originalMeasurements) {
	this.resetCarDemand();
	if(originalMeasurements==null) {
		this.emptyMeasurements=true;
		this.measurementsToUpdate=Measurements.createMeasurements(this.timeBeans);
	}else {
		this.measurementsToUpdate=originalMeasurements.clone();
		this.measurementsToUpdate.resetMeasurements();
	}


	//Checking and updating for the parameters 
	for(Entry<String,Double> e:this.Params.entrySet()) {
		if(!params.containsKey(e.getKey())) {
			params.put(e.getKey(), e.getValue());
		}
	}

	//Checking and updating for the analytical model parameters
	for(Entry<String,Double> e:this.AnalyticalModelInternalParams.entrySet()) {
		if(!anaParams.containsKey(e.getKey())) {
			anaParams.put(e.getKey(), e.getValue());
		}
	}
	for(String timeBeanId:this.timeBeans.keySet()) {
		this.singleTimeBeanTA(params, anaParams, timeBeanId);
	}
	

	//Collecting the Link Flows

	if(this.emptyMeasurements==true) {
		for(String timeBeanId:this.timeBeans.keySet()) {
			double count=0;
			for(Link link:this.networks.get(timeBeanId).getLinks().values()) {
				if(!link.getAllowedModes().contains("train") && !link.getId().toString().contains("stop")) {
				count=((AnalyticalModelLink) link).getLinkAADTVolume();
				Id<Measurement> mId=Id.create(link.getId().toString(), Measurement.class);
				Measurement m=null;
				if((m=this.measurementsToUpdate.getMeasurements().get(mId))==null) {
					this.measurementsToUpdate.createAnadAddMeasurement(mId.toString(), MeasurementType.linkVolume);
					m=this.measurementsToUpdate.getMeasurements().get(mId);
					ArrayList<Id<Link>> linkList=new ArrayList<>();
					linkList.add(link.getId());
					m.setAttribute(Measurement.linkListAttributeName, linkList);
				}
				m.putVolume(timeBeanId, count);
				}
			}

		}

	}else {
		for(Measurement m:this.measurementsToUpdate.getMeasurementsByType().get(MeasurementType.linkVolume)) {
			for(String timeBeanId:m.getVolumes().keySet()) {
				double count=0;
				for(Id<Link> linkId:(ArrayList<Id<Link>>)m.getAttribute(Measurement.linkListAttributeName)) {
					count+=((AnalyticalModelLink) this.networks.get(timeBeanId).getLinks().get(linkId)).getLinkAADTVolume();
				}
				m.putVolume(timeBeanId, count);
			}
		}
	}


	//For now shut down for null Measurements
	//collect pt occupancy
	for(Measurement m:this.measurementsToUpdate.getMeasurementsByType().get(MeasurementType.averagePTOccumpancy)) {
		for(String timeBeanId:m.getVolumes().keySet()) {
			Id<Link>linkId=((ArrayList<Id<Link>>)m.getAttribute(Measurement.linkListAttributeName)).get(0);
			double occupancy=((CNLLink)this.networks.get(timeBeanId).getLinks().get(linkId)).getLinkTransitPassenger()/this.totalPtCapacityOnLink.get(timeBeanId).get(linkId);
			m.putVolume(timeBeanId, occupancy);
		}
	}

	//collect smartCard Entry
	if(this.emptyMeasurements==false) {
		Map<String,Map<String,Double>>entryCount=new HashMap<>();//First string is lineid+routeid+entryStopId second string is volume key
		for(Measurement m:this.measurementsToUpdate.getMeasurementsByType().get(MeasurementType.smartCardEntry)) {
			String key=m.getAttribute(Measurement.transitLineAttributeName)+"___"+m.getAttribute(Measurement.transitRouteAttributeName)+"___"+m.getAttribute(Measurement.transitBoardingStopAtrributeName);
			//System.out.println();
			entryCount.put(key, new HashMap<>());
			for(String s:m.getVolumes().keySet()) {
				entryCount.get(key).put(s, 0.);
			}
		}

		for(String timeBeanId:this.transitLinks.keySet()) {
			for(TransitLink trl:this.transitLinks.get(timeBeanId).values()) {
				if(trl instanceof TransitDirectLink) {
					TransitDirectLink trdl=(TransitDirectLink)trl;
					String key= trdl.getLineId()+"___"+trdl.getRouteId()+"___"+trdl.getStartStopId();
					if(entryCount.containsKey(key) && entryCount.get(key).containsKey(timeBeanId)) {
						entryCount.get(key).put(timeBeanId, entryCount.get(key).get(timeBeanId)+trl.getPassangerCount());
					}
				}
			}
		}

		for(Measurement m:this.measurementsToUpdate.getMeasurementsByType().get(MeasurementType.smartCardEntry)) {
			String key=m.getAttribute(Measurement.transitLineAttributeName)+"___"+m.getAttribute(Measurement.transitRouteAttributeName)+"___"+m.getAttribute(Measurement.transitBoardingStopAtrributeName);
			for(String timeBeanId:m.getVolumes().keySet()) {
				m.putVolume(timeBeanId, entryCount.get(key).get(timeBeanId));
			}
		}
	}else {
		for(String timeBeanId:this.transitLinks.keySet()) {
			for(TransitLink trl:this.transitLinks.get(timeBeanId).values()) {
				if(trl instanceof TransitDirectLink) {
					TransitDirectLink trdl=(TransitDirectLink)trl;
					String key= trdl.getLineId()+"___"+trdl.getRouteId()+"___"+trdl.getStartStopId();
					Id<TransitLine> lineId=Id.create(trdl.getLineId(),TransitLine.class);
					Id<TransitRoute>routeId=Id.create(trdl.getRouteId(),TransitRoute.class);
					String mode=this.ts.getTransitLines().get(lineId).getRoutes().get(routeId).getTransportMode();
					Id<Measurement>mId=Id.create(key, Measurement.class);
					Measurement m=null;
					if((m=this.measurementsToUpdate.getMeasurements().get(mId))==null) {
						this.measurementsToUpdate.createAnadAddMeasurement(key, MeasurementType.smartCardEntry);
						m=this.measurementsToUpdate.getMeasurements().get(mId);
						m.setAttribute(Measurement.transitLineAttributeName, trdl.getLineId());
						m.setAttribute(Measurement.transitRouteAttributeName, trdl.getRouteId());
						m.setAttribute(Measurement.transitBoardingStopAtrributeName, trdl.getStartStopId());
						m.setAttribute(Measurement.transitModeAttributeName, mode);
					}
					Double oldVolume=null;
					if((oldVolume=m.getVolumes().get(timeBeanId))==null) {
						m.putVolume(timeBeanId, trl.getPassangerCount());
					}else {
						m.putVolume(timeBeanId, oldVolume+trl.getPassangerCount());
					}
				}
			}
		}
	}

	//Collect smart card entry and exit through farelink
	if(this.emptyMeasurements==false) {
		for(AnalyticalModelODpair odpair:this.odPairs.getODpairset().values()) {
			for(String timeBeanId:this.timeBeans.keySet()) {
				if(odpair.getTrRoutes(timeBeanId)!=null && this.Demand.get(timeBeanId).get(odpair.getODpairId())!=0) {
					for(AnalyticalModelTransitRoute tr:odpair.getTrRoutes(timeBeanId)) {
						for(FareLink fl:((CNLTransitRoute)tr).getFareLinks()) {
							Id<Measurement> mId=Id.create(fl.toString(), Measurement.class);
							Measurement m=this.measurementsToUpdate.getMeasurements().get(mId);
							if(m.getVolumes().containsKey(timeBeanId)) {
								m.putVolume(timeBeanId, m.getVolumes().get(timeBeanId)+odpair.getTrRouteFlow().get(timeBeanId).get(tr.getTrRouteId()));
							}

						}
					}
				}
			}
		}
	}else {

		for(AnalyticalModelODpair odpair:this.odPairs.getODpairset().values()) {
			for(String timeBeanId:this.timeBeans.keySet()) {
				if(odpair.getTrRoutes(timeBeanId)!=null && this.Demand.get(timeBeanId).get(odpair.getODpairId())!=0) {
					for(AnalyticalModelTransitRoute tr:odpair.getTrRoutes(timeBeanId)) {
						for(FareLink fl:((CNLTransitRoute)tr).getFareLinks()) {
							Id<Measurement> mId=Id.create(fl.toString(), Measurement.class);
							Measurement m=null;
							if((m=this.measurementsToUpdate.getMeasurements().get(mId))==null) {
								this.measurementsToUpdate.createAnadAddMeasurement(mId.toString(), MeasurementType.smartCardEntryAndExit);
								m=this.measurementsToUpdate.getMeasurements().get(mId);
								m.setAttribute(Measurement.FareLinkAttributeName, fl);
							}
							if(m.getVolumes().containsKey(timeBeanId)) {
								m.putVolume(timeBeanId, m.getVolumes().get(timeBeanId)+odpair.getTrRouteFlow().get(timeBeanId).get(tr.getTrRouteId()));
							}else {
								m.putVolume(timeBeanId, odpair.getTrRouteFlow().get(timeBeanId).get(tr.getTrRouteId()));
							}

						}
					}
				}
			}
		}
	}




	//new OdInfoWriter("toyScenario/ODInfo/odInfo",this.timeBeans).writeOdInfo(this.getOdPairs(), getDemand(), getCarDemand(), inputParams, inputAnaParams);
	return this.measurementsToUpdate;
}

//public Map<String, Map<Id<Link>, Map<String, Double>>> getLinkGradient() {
public Map<String, Map<Id<Link>, double[]>> getLinkGradient() {
	return linkGradient;
}



//public Map<String, Map<Id<TransitLink>, Map<String, Double>>> getTrLinkGradient() {
public Map<String, Map<Id<TransitLink>, double[]>> getTrLinkGradient() {
	return trLinkGradient;
}



//public Map<String, Map<String, Map<String, Double>>> getFareLinkGradient() {
public Map<String, Map<String, double[]>> getFareLinkGradient() {
	return fareLinkGradient;
}

public Set<String> getGradientKeys() {
	return gradientKeys;
}

public CNLODpairs getOdPairs() {
	return odPairs;
}

public void setOdPairs(CNLODpairs odPairs) {
	this.odPairs = odPairs;
}

public MapToArray<String> getGradientArray() {
	return gradientArray;
}


private void applyODBasedGradeintClipping(Map<Id<AnalyticalModelRoute>,double[]>routeGrad,Map<Id<AnalyticalModelTransitRoute>,double[]>trRouteGrad,Id<AnalyticalModelODpair> odId,String timeBeanId) {
	RealVector delta = MatrixUtils.createRealVector(this.odParameterIncidence.get(timeBeanId).get(odId));
	RealVector norm = MatrixUtils.createRealVector(new double[delta.toArray().length]);
	for(double[] g:routeGrad.values())norm = norm.add(MatrixUtils.createRealVector(g).map(k->Math.abs(k)));
	for(double[] g:trRouteGrad.values())norm = norm.add(MatrixUtils.createRealVector(g).map(k->Math.abs(k)));
	RealVector demand  =  delta.mapAdd(2).mapMultiply(this.Demand.get(timeBeanId).get(odId));
	RealVector multiplier = demand.ebeDivide(norm);
	RealVector m = demand.subtract(norm).map(k->k>0?1:0).ebeMultiply(multiplier).map(k->k==0?1:k);
	multiplier = multiplier.ebeDivide(m);
	
	for(Entry<Id<AnalyticalModelRoute>,double[]> g:routeGrad.entrySet())g.setValue(MatrixUtils.createRealVector(g.getValue()).ebeMultiply(multiplier).toArray());
	for(Entry<Id<AnalyticalModelTransitRoute>,double[]> g:trRouteGrad.entrySet())g.setValue(MatrixUtils.createRealVector(g.getValue()).ebeMultiply(multiplier).toArray());
}

private void applyODBasedGradeintClipping(Set<Id<AnalyticalModelRoute>>routeIds,Set<Id<AnalyticalModelTransitRoute>>trRouteIds,Id<AnalyticalModelODpair> odId,String timeBeanId) {
	RealVector delta = MatrixUtils.createRealVector(this.odParameterIncidence.get(timeBeanId).get(odId));
	RealVector norm = MatrixUtils.createRealVector(new double[delta.toArray().length]);
	for(Id<AnalyticalModelRoute> rId:routeIds)norm = norm.add(MatrixUtils.createRealVector(this.routeFlowGradient.get(timeBeanId).get(rId)).map(k->Math.abs(k)));
	for(Id<AnalyticalModelTransitRoute> rId:trRouteIds)norm = norm.add(MatrixUtils.createRealVector(this.trRouteFlowGradient.get(timeBeanId).get(rId)).map(k->Math.abs(k)));
	RealVector demand  =  delta.mapAdd(2).mapMultiply(this.Demand.get(timeBeanId).get(odId));
	RealVector multiplier = demand.ebeDivide(norm);
	RealVector m = demand.subtract(norm).map(k->k>0?1:0).ebeMultiply(multiplier).map(k->k==0?1:k);
	multiplier = multiplier.ebeDivide(m);
	for(Id<AnalyticalModelRoute> rId:routeIds)this.routeFlowGradient.get(timeBeanId).put(rId, MatrixUtils.createRealVector(this.routeFlowGradient.get(timeBeanId).get(rId)).ebeMultiply(multiplier).toArray());
	for(Id<AnalyticalModelTransitRoute> rId:trRouteIds)this.trRouteFlowGradient.get(timeBeanId).put(rId, MatrixUtils.createRealVector(this.trRouteFlowGradient.get(timeBeanId).get(rId)).ebeMultiply(multiplier).toArray());
}

private void applyVerticalGradeintClipping(Set<Id<AnalyticalModelRoute>>routeIds,Set<Id<AnalyticalModelTransitRoute>>trRouteIds,Id<AnalyticalModelODpair> odId,String timeBeanId) {
	RealVector delta = MatrixUtils.createRealVector(this.odParameterIncidence.get(timeBeanId).get(odId));
	RealVector norm = MatrixUtils.createRealVector(new double[delta.toArray().length]);
	for(Id<AnalyticalModelRoute> rId:routeIds)norm = norm.add(MatrixUtils.createRealVector(this.routeFlowGradient.get(timeBeanId).get(rId)).map(k->Math.abs(k)));
	for(Id<AnalyticalModelTransitRoute> rId:trRouteIds)norm = norm.add(MatrixUtils.createRealVector(this.trRouteFlowGradient.get(timeBeanId).get(rId)).map(k->Math.abs(k)));
	RealVector demand  =  delta.mapAdd(2).mapMultiply(this.Demand.get(timeBeanId).get(odId));
	RealVector multiplier = demand.ebeDivide(norm);
	RealVector m = demand.subtract(norm).map(k->k>0?1:0).ebeMultiply(multiplier).map(k->k==0?1:k);
	multiplier = multiplier.ebeDivide(m);
	for(Id<AnalyticalModelRoute> rId:routeIds)this.routeFlowGradient.get(timeBeanId).put(rId, MatrixUtils.createRealVector(this.routeFlowGradient.get(timeBeanId).get(rId)).ebeMultiply(multiplier).toArray());
	for(Id<AnalyticalModelTransitRoute> rId:trRouteIds)this.trRouteFlowGradient.get(timeBeanId).put(rId, MatrixUtils.createRealVector(this.trRouteFlowGradient.get(timeBeanId).get(rId)).ebeMultiply(multiplier).toArray());
}

public static double[] findAbsMax(double[]a,double[]b){
	double[] out = new double[a.length];
	 for(int i = 0;i<a.length;i++) {
		out[i] = Double.max(Math.abs(a[i]),Math.abs(b[i]));
	}
	return out; 
}

private void scaleBackGradients() {
	RealVector m = MatrixUtils.createRealVector(this.gradMultiplier);
	this.routeFlowGradient.entrySet().forEach(t->{
		t.getValue().entrySet().forEach(d->{
			RealVector dd = MatrixUtils.createRealVector(d.getValue()).ebeMultiply(m); 
			if(dd.isInfinite()||dd.isNaN()) {
				logger.debug("gradient infinite or nan!!!");
			}
			d.setValue(dd.toArray());
//			for(int i = 0;i<d.length;i++)
//				d[i] = d[i]*this.gradMultiplier[i];
		});
	});
	
	this.trRouteFlowGradient.entrySet().forEach(t->{
		t.getValue().entrySet().forEach(d->{
			RealVector dd = MatrixUtils.createRealVector(d.getValue()).ebeMultiply(m); 
			if(dd.isInfinite()||dd.isNaN()) {
				logger.debug("gradient infinite or nan!!!");
			}
			d.setValue(dd.toArray());
//			for(int i = 0;i<d.length;i++)
//				d[i] = d[i]*this.gradMultiplier[i];
		});
	});
	
	this.linkGradient.entrySet().forEach(t->{
		t.getValue().entrySet().forEach(d->{
			RealVector dd = MatrixUtils.createRealVector(d.getValue()).ebeMultiply(m); 
			if(dd.isInfinite()||dd.isNaN()) {
				logger.debug("gradient infinite or nan!!!");
			}
			d.setValue(dd.toArray());
//			for(int i = 0;i<d.length;i++)
//				d[i] = d[i]*this.gradMultiplier[i];
		});
	});
	
	this.trLinkGradient.entrySet().forEach(t->{
		t.getValue().entrySet().forEach(d->{
			RealVector dd = MatrixUtils.createRealVector(d.getValue()).ebeMultiply(m); 
			if(dd.isInfinite()||dd.isNaN()) {
				logger.debug("gradient infinite or nan!!!");
			}
			d.setValue(dd.toArray());
//			for(int i = 0;i<d.length;i++)
//				d[i] = d[i]*this.gradMultiplier[i];
		});
	});
	
	this.linkTTGradient.entrySet().forEach(t->{
		t.getValue().entrySet().forEach(d->{
			RealVector dd = MatrixUtils.createRealVector(d.getValue()).ebeMultiply(m); 
			if(dd.isInfinite()||dd.isNaN()) {
				logger.debug("gradient infinite or nan!!!");
			}
			d.setValue(dd.toArray());
//			for(int i = 0;i<d.length;i++)
//				d[i] = d[i]*this.gradMultiplier[i];
		});
	});
	
	this.trLinkTTGradient.entrySet().forEach(t->{
		t.getValue().entrySet().forEach(d->{
			RealVector dd = MatrixUtils.createRealVector(d.getValue()).ebeMultiply(m); 
			if(dd.isInfinite()||dd.isNaN()) {
				logger.debug("gradient infinite or nan!!!");
			}
			d.setValue(dd.toArray());
//			for(int i = 0;i<d.length;i++)
//				d[i] = d[i]*this.gradMultiplier[i];
		});
	});
	
	this.fareLinkGradient.entrySet().forEach(t->{
		t.getValue().entrySet().forEach(d->{
			RealVector dd = MatrixUtils.createRealVector(d.getValue()).ebeMultiply(m); 
			if(dd.isInfinite()||dd.isNaN()) {
				logger.debug("gradient infinite or nan!!!");
			}
			d.setValue(dd.toArray());
//			for(int i = 0;i<d.length;i++)
//				d[i] = d[i]*this.gradMultiplier[i];
		});
	});
}

private void scaleGradients() {
	RealVector m = MatrixUtils.createRealVector(this.gradMultiplier);
	this.routeFlowGradient.entrySet().forEach(t->{
		t.getValue().entrySet().forEach(d->{
			RealVector dd = MatrixUtils.createRealVector(d.getValue()).ebeDivide(m); 
			if(dd.isInfinite()||dd.isNaN()) {
				logger.debug("gradient infinite or nan!!!");
			}
			d.setValue(dd.toArray());
//			for(int i = 0;i<d.length;i++)
//				d[i] = d[i]*this.gradMultiplier[i];
		});
	});
	
	this.trRouteFlowGradient.entrySet().forEach(t->{
		t.getValue().entrySet().forEach(d->{
			RealVector dd = MatrixUtils.createRealVector(d.getValue()).ebeDivide(m); 
			if(dd.isInfinite()||dd.isNaN()) {
				logger.debug("gradient infinite or nan!!!");
			}
			d.setValue(dd.toArray());
//			for(int i = 0;i<d.length;i++)
//				d[i] = d[i]*this.gradMultiplier[i];
		});
	});
	
	this.linkGradient.entrySet().forEach(t->{
		t.getValue().entrySet().forEach(d->{
			RealVector dd = MatrixUtils.createRealVector(d.getValue()).ebeDivide(m); 
			if(dd.isInfinite()||dd.isNaN()) {
				logger.debug("gradient infinite or nan!!!");
			}
			d.setValue(dd.toArray());
//			for(int i = 0;i<d.length;i++)
//				d[i] = d[i]*this.gradMultiplier[i];
		});
	});
	
	this.trLinkGradient.entrySet().forEach(t->{
		t.getValue().entrySet().forEach(d->{
			RealVector dd = MatrixUtils.createRealVector(d.getValue()).ebeDivide(m); 
			if(dd.isInfinite()||dd.isNaN()) {
				logger.debug("gradient infinite or nan!!!");
			}
			d.setValue(dd.toArray());
//			for(int i = 0;i<d.length;i++)
//				d[i] = d[i]*this.gradMultiplier[i];
		});
	});
	
	this.linkTTGradient.entrySet().forEach(t->{
		t.getValue().entrySet().forEach(d->{
			RealVector dd = MatrixUtils.createRealVector(d.getValue()).ebeDivide(m); 
			if(dd.isInfinite()||dd.isNaN()) {
				logger.debug("gradient infinite or nan!!!");
			}
			d.setValue(dd.toArray());
//			for(int i = 0;i<d.length;i++)
//				d[i] = d[i]*this.gradMultiplier[i];
		});
	});
	
	this.trLinkTTGradient.entrySet().forEach(t->{
		t.getValue().entrySet().forEach(d->{
			RealVector dd = MatrixUtils.createRealVector(d.getValue()).ebeDivide(m); 
			if(dd.isInfinite()||dd.isNaN()) {
				logger.debug("gradient infinite or nan!!!");
			}
			d.setValue(dd.toArray());
//			for(int i = 0;i<d.length;i++)
//				d[i] = d[i]*this.gradMultiplier[i];
		});
	});
	
	this.fareLinkGradient.entrySet().forEach(t->{
		t.getValue().entrySet().forEach(d->{
			RealVector dd = MatrixUtils.createRealVector(d.getValue()).ebeDivide(m); 
			if(dd.isInfinite()||dd.isNaN()) {
				logger.debug("gradient infinite or nan!!!");
			}
			d.setValue(dd.toArray());
//			for(int i = 0;i<d.length;i++)
//				d[i] = d[i]*this.gradMultiplier[i];
		});
	});
}
}
