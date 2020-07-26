package AnalyticalModel;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.Map.Entry;
import java.util.concurrent.ConcurrentHashMap;

import org.apache.log4j.Logger;
import org.matsim.api.core.v01.Id;
import org.matsim.api.core.v01.Scenario;
import org.matsim.api.core.v01.network.Link;
import org.matsim.api.core.v01.network.Network;
import org.matsim.api.core.v01.population.Population;
import org.matsim.core.config.Config;
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
import ust.hk.praisehk.metamodelcalibration.analyticalModelImpl.CNLLink;
import ust.hk.praisehk.metamodelcalibration.analyticalModelImpl.CNLNetwork;
import ust.hk.praisehk.metamodelcalibration.analyticalModelImpl.CNLODpairs;
import ust.hk.praisehk.metamodelcalibration.analyticalModelImpl.CNLSUEModel;
import ust.hk.praisehk.metamodelcalibration.analyticalModelImpl.CNLTransitRoute;

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


//Internal database for the utility, mode choice and utility

private Map<String,Map<Id<AnalyticalModelRoute>,Double>> routeUtilities = new HashMap<>();
private Map<String,Map<Id<AnalyticalModelTransitRoute>,Double>> trRouteUtilities = new HashMap<>();
private Map<String,Map<Id<AnalyticalModelODpair>,Double>> expectedMaximumCarUtility = new HashMap<>();
private Map<String,Map<Id<AnalyticalModelODpair>,Double>> expectedMaximumTrUtility = new HashMap<>();

private Map<String,Map<Id<AnalyticalModelRoute>,Double>> routeProb = new HashMap<>();
private Map<String,Map<Id<AnalyticalModelTransitRoute>,Double>> trRouteProb = new HashMap<>();

private Map<String,Map<Id<AnalyticalModelODpair>,Double>> carProbability = new HashMap<>();


//This are needed for output generation 




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

public void generateRoutesAndOD(Population population,Network network,TransitSchedule transitSchedule,
		Scenario scenario,Map<String,FareCalculator> fareCalculator) {
	this.scenario = scenario;
	this.population = population;
	//System.out.println("");
	this.odPairs = new CNLODpairs(network,population,transitSchedule,scenario,this.timeBeans);
//	Config odConfig=ConfigUtils.createConfig();
//	odConfig.network().setInputFile("data/odNetwork.xml");
	this.odPairs.generateOdSpecificRouteKeys();
	Network odNetwork=NetworkUtils.readNetwork("data/tpusbNetwork.xml");//This is for creating ODpairs based on TPUSBs
	this.odPairs.generateODpairsetSubPop(odNetwork);//This network has priority over the constructor network. This allows to use a od pair specific network 
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
	
	for(String timeBeanId:this.timeBeans.keySet()) {
		this.consecutiveSUEErrorIncrease.put(timeBeanId, 0.);
		this.Demand.put(timeBeanId, new HashMap<>(this.odPairs.getdemand(timeBeanId)));
		for(Id<AnalyticalModelODpair> odId:this.Demand.get(timeBeanId).keySet()) {
			double totalDemand=this.Demand.get(timeBeanId).get(odId);
			this.carDemand.get(timeBeanId).put(odId, 0.5*totalDemand);
			
			AnalyticalModelODpair odpair=this.odPairs.getODpairset().get(odId);
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

}

private LinkedHashMap<String,Double> handleBasicParams(LinkedHashMap<String,Double> params, String subPopulation, Config config){
	LinkedHashMap<String,Double> newParams = new LinkedHashMap<>();
	// Handle the original params first
	for(String s:params.keySet()) {
		if(subPopulation!=null && (s.contains(subPopulation)||s.contains("All"))) {
			newParams.put(s.split(" ")[1],params.get(s));
		}else if (subPopulation == null) {
			newParams.put(s, params.get(s));
		}
	}
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
	
	return newParams;
}

public Measurements perFormSUE(LinkedHashMap<String, Double> params,Measurements originalMeasurements) {
	ODUtils.applyODPairMultiplier(this.Demand, params);
	return this.performAssignment(params, this.AnalyticalModelInternalParams,originalMeasurements);
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

private SUEModelOutput performAssignment( LinkedHashMap<String,Double> params, LinkedHashMap<String,Double> anaParams) {
	SUEModelOutput flow = null;
//	for(int counter = 1; counter < this.maxIter; counter++) {
//		if(counter == 1)this.createIncidenceMaps(population);
//		
//		flow  = this.performNetworkLoading(population, params, anaParams,counter);//link incidences are ready after this step
//		this.caclulateGradient(population, counter, params, anaParams);
//		boolean shouldStop = this.updateVolume(flow.getLinkVolume(), flow.getLinkTransitVolume(), counter);//transit link dependencies are ready after this step
//		if(shouldStop) {
//			break;
//		}
//	}
//	flow.setMaaSPackageUsage(this.calculateMaaSPackageUsage());
	return flow;
}

public void singleTimeBeanTA(LinkedHashMap<String, Double> params,LinkedHashMap<String,Double> anaParams,String timeBeanId) {
	Map<Id<TransitLink>, Double> linkTransitVolume;
	Map<Id<Link>,Double> linkCarVolume;
	boolean shouldStop=false;
	
	for(int i=1;i<500;i++) {
		//for(this.car)
		//ConcurrentHashMap<String,HashMap<Id<CNLODpair>,Double>>demand=this.Demand;
		linkCarVolume=this.performCarNetworkLoading(timeBeanId,i,params,anaParams);
		linkTransitVolume=this.performTransitNetworkLoading(timeBeanId,i,params,anaParams);
		shouldStop=this.CheckConvergence(linkCarVolume, linkTransitVolume, this.tollerance, timeBeanId,i);
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
			//collect travel time
			if(this.measurementsToUpdate!=null) {
				List<Measurement>ms= this.measurementsToUpdate.getMeasurementsByType().get(MeasurementType.linkTravelTime);
				for(Measurement m:ms) {
					if(m.getVolumes().containsKey(timeBeanId)) {
						m.putVolume(timeBeanId, ((CNLLink)this.networks.get(timeBeanId).getLinks().get(((ArrayList<Id<Link>>)m.getAttribute(Measurement.linkListAttributeName)).get(0))).getLinkTravelTime(this.timeBeans.get(timeBeanId),
						params, anaParams));
					}
				}
			}
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
			
			break;
			}
		this.performModalSplit(params, anaParams, timeBeanId);
		
	}
	
	
}

protected HashMap<Id<Link>,Double> NetworkLoadingCarSingleOD(Id<AnalyticalModelODpair> ODpairId,String timeBeanId,double counter,LinkedHashMap<String,Double> params, LinkedHashMap<String, Double> anaParams){
	
	AnalyticalModelODpair odpair=this.odPairs.getODpairset().get(ODpairId);
	String subPopulation = odpair.getSubPopulation();
	
	
	List<AnalyticalModelRoute> routes=odpair.getRoutes();
	Map<Id<AnalyticalModelRoute>,Double> routeFlows=new HashMap<>();
	HashMap<Id<Link>,Double> linkFlows=new HashMap<>();
	
	params = this.handleBasicParams(params, subPopulation, this.scenario.getConfig());
	
	//double totalUtility=0;
	
	//Calculating route utility for all car routes inside one OD pair.
	
	//HashMap<Id<AnalyticalModelRoute>,Double> oldUtility=new HashMap<>();
	Map<Id<AnalyticalModelRoute>,Double> utility=this.routeUtilities.get(timeBeanId);
	double maxUtil = Double.MIN_VALUE;
	double denominator = 0;
	for(AnalyticalModelRoute r:routes){
		double u=0;
		
		u=r.calcRouteUtility(params, anaParams,this.networks.get(timeBeanId),this.timeBeans.get(timeBeanId));
		u=u+Math.log(odpair.getAutoPathSize().get(r.getRouteId()));//adding the path size term
		if(maxUtil<u)maxUtil = u;
		utility.put(r.getRouteId(), u);
		odpair.updateRouteUtility(r.getRouteId(), u,timeBeanId);// Not sure if this is needed
		
		//This Check is to make sure the exp(utility) do not go to infinity.
		if(u>300||u<-300) {
			logger.error("utility is either too small or too large. Increase or decrease the link miu accordingly. The utility is "+u+" for route "+r.getRouteId());
			//throw new IllegalArgumentException("stop!!!");
		}
	}
	for(AnalyticalModelRoute r:routes){
		denominator += Math.exp(utility.get(r.getRouteId())-maxUtil);
	}
	
	//This is the route flow split
	
	for(AnalyticalModelRoute r:routes){
		double u=utility.get(r.getRouteId());
		double demand = this.carDemand.get(timeBeanId).get(ODpairId);
		double prob = (u-maxUtil)/denominator;
		this.routeProb.get(timeBeanId).put(r.getRouteId(), prob);
		double flow=prob*demand;
		//For testing purpose, can be removed later
		if(flow==Double.NaN||flow==Double.POSITIVE_INFINITY) {
			logger.error("The flow is NAN. This can happen for a number of reasons. Mostly is total utility of all the routes in a OD pair is zero");
			throw new IllegalArgumentException("Wait!!!!Error!!!!");
		}
		routeFlows.put(r.getRouteId(),flow);
		odpair.getRouteFlow().get(timeBeanId).put(r.getRouteId(), flow);	
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
protected HashMap<Id<TransitLink>,Double> NetworkLoadingTransitSingleOD(Id<AnalyticalModelODpair> ODpairId,String timeBeanId,int counter,LinkedHashMap<String,Double> params, LinkedHashMap<String, Double> anaParams){
	
	AnalyticalModelODpair odpair=this.odPairs.getODpairset().get(ODpairId);
	List<AnalyticalModelTransitRoute> routes=odpair.getTrRoutes(timeBeanId);
	
	HashMap<Id<AnalyticalModelTransitRoute>,Double> routeFlows=new HashMap<>();
	HashMap<Id<TransitLink>,Double> linkFlows=new HashMap<>();
	
	Map<Id<AnalyticalModelTransitRoute>,Double> utility=this.trRouteUtilities.get(timeBeanId);
	
	if(routes!=null && routes.size()!=0) {
		double maxUtil = Double.MIN_VALUE;
		double denominator = 0;
		for(AnalyticalModelTransitRoute r:routes){
			double u=0;

			u=r.calcRouteUtility(params, anaParams,
					this.networks.get(timeBeanId),this.fareCalculator,null,this.timeBeans.get(timeBeanId));
			u+=Math.log(odpair.getTrPathSize().get(timeBeanId).get(r.getTrRouteId()));//adding the path size term
			if(maxUtil<u)maxUtil = u;
			if(u==Double.NaN) {
				logger.error("The flow is NAN. This can happen for a number of reasons. Mostly is total utility of all the routes in a OD pair is zero");
				throw new IllegalArgumentException("Utility is NAN!!!");
			}

			if(u>300) {
				logger.warn("STOP!!!Utility is too large >300");
			}
			odpair.updateTrRouteUtility(r.getTrRouteId(), u,timeBeanId);
			utility.put(r.getTrRouteId(), u);

		}
		for(AnalyticalModelTransitRoute r:routes){
			denominator += Math.exp(utility.get(r.getTrRouteId())-maxUtil);
		}

		for(AnalyticalModelTransitRoute r:routes){
			double totalDemand=this.Demand.get(timeBeanId).get(ODpairId);
			double carDemand=this.carDemand.get(timeBeanId).get(ODpairId);
			double q=(totalDemand-carDemand);
			String id=null;
			double u=utility.get(r.getTrRouteId());
			double prob = (u-maxUtil)/denominator;
			this.trRouteProb.get(timeBeanId).put(r.getTrRouteId(), prob);
			double flow=q*prob;
			if(Double.isNaN(flow)||flow==Double.POSITIVE_INFINITY||flow==Double.NEGATIVE_INFINITY) {
				logger.error("The flow is NAN. This can happen for a number of reasons. Mostly is total utility of all the routes in a OD pair is zero");
				throw new IllegalArgumentException("Error!!!!");
			}
			routeFlows.put(r.getTrRouteId(),flow);
			odpair.getTrRouteFlow().get(timeBeanId).put(r.getTrRouteId(), flow);		
		}

	}
	
	Set<Id<TransitLink>>linksets=this.odPairs.getODpairset().get(ODpairId).getTrLinkIncidence().keySet();
	for(Id<TransitLink> linkId:linksets){
		if(this.transitLinks.get(timeBeanId).containsKey(linkId)) {
		double linkflow=0;
		List<AnalyticalModelTransitRoute>incidence=this.odPairs.getODpairset().get(ODpairId).getTrLinkIncidence().get(linkId);
		for(AnalyticalModelTransitRoute r:incidence){
			List<AnalyticalModelTransitRoute> routesFromOd=routes;
			
			if(CNLSUEModel.routeContain(routesFromOd, r)) {
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
			double totalDemand=this.Demand.get(timeBeanId).get(odpair.getODpairId());
			double carDemand=this.carDemand.get(timeBeanId).get(odpair.getODpairId());
			if((totalDemand-carDemand)!=0) {
				linkTransitVolumes.add(this.NetworkLoadingTransitSingleOD(odpair.getODpairId(),timeBeanId,counter,params,anaParams));
			}
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

@SuppressWarnings("unchecked")
protected boolean UpdateLinkVolume(Map<Id<Link>,Double> linkVolume,Map<Id<TransitLink>,Double> transitlinkVolume,int counter,String timeBeanId){
	double squareSum=0;
	double flowSum=0;
	double linkSum=0;
	if(counter==1) {
		this.beta.get(timeBeanId).clear();
		//this.error.clear();
		this.beta.get(timeBeanId).add(1.);
	}else {
		if(error.get(timeBeanId).get(counter-1)<error.get(timeBeanId).get(counter-2)) {
			beta.get(timeBeanId).add(beta.get(timeBeanId).get(counter-2)+this.gammaMSA);
		}else {
			this.consecutiveSUEErrorIncrease.put(timeBeanId, this.consecutiveSUEErrorIncrease.get(timeBeanId)+1);
			beta.get(timeBeanId).add(beta.get(timeBeanId).get(counter-2)+this.alphaMSA);
			
		}
	}
	
	for(Id<Link> linkId:linkVolume.keySet()){
		double newVolume=linkVolume.get(linkId);
		double oldVolume=((AnalyticalModelLink) this.networks.get(timeBeanId).getLinks().get(linkId)).getLinkCarVolume();
		flowSum+=oldVolume;
		double update;
		double counterPart=1/beta.get(timeBeanId).get(counter-1);
		//counterPart=1./counter;
		update=counterPart*(newVolume-oldVolume);
		if(oldVolume!=0) {
			if(Math.abs(update)/oldVolume*100>this.tolleranceLink) {
				linkSum+=1;
			}
		}
		squareSum+=update*update;
		((AnalyticalModelLink) this.networks.get(timeBeanId).getLinks().get(linkId)).addLinkCarVolume(update);
	}
	for(Id<TransitLink> trlinkId:transitlinkVolume.keySet()){
		//System.out.println("testing");
		double newVolume=transitlinkVolume.get(trlinkId);
		TransitLink trl=this.transitLinks.get(timeBeanId).get(trlinkId);
		double oldVolume=trl.getPassangerCount();
		double update;
		double counterPart=1/beta.get(timeBeanId).get(counter-1);
		
		update=counterPart*(newVolume-oldVolume);
		if(oldVolume!=0) {
			if(Math.abs(update)/oldVolume*100>this.tolleranceLink) {
				linkSum+=1;
			}
			
		}
		squareSum+=update*update;
		this.transitLinks.get(timeBeanId).get(trlinkId).addPassanger(update,this.networks.get(timeBeanId));
	
	}
	squareSum=Math.sqrt(squareSum);
	if(counter==1) {
		this.error1.get(timeBeanId).clear();
	}
	error1.get(timeBeanId).add(squareSum);
	
	if(squareSum<this.tollerance) {
		return true;
		
	}else {
		return false;
	}
}

/**
 * This method will check for the convergence and also create the error term required for MSA
 * @param linkVolume
 * @param tollerance
 * @return
 */
protected boolean CheckConvergence(Map<Id<Link>,Double> linkVolume,Map<Id<TransitLink>,Double> transitlinkVolume, double tollerance,String timeBeanId,int counter){
	double linkBelow1=0;
	double squareSum=0;
	double sum=0;
	double error=0;
	for(Id<Link> linkid:linkVolume.keySet()){
		if(linkVolume.get(linkid)==0) {
			error=0;
		}else {
			double currentVolume=((AnalyticalModelLink) this.networks.get(timeBeanId).getLinks().get(linkid)).getLinkCarVolume();
			double newVolume=linkVolume.get(linkid);
			error=Math.pow((currentVolume-newVolume),2);
			if(error==Double.POSITIVE_INFINITY||error==Double.NEGATIVE_INFINITY) {
				throw new IllegalArgumentException("Error is infinity!!!");
			}
			if(error/newVolume*100>tollerance) {					
				sum+=1;
			}
			if(error<1) {
				linkBelow1++;
			}
		}
		
		squareSum+=error;
		if(squareSum==Double.POSITIVE_INFINITY||squareSum==Double.NEGATIVE_INFINITY) {
			throw new IllegalArgumentException("error is infinity!!!");
		}
	}
	for(Id<TransitLink> transitlinkid:transitlinkVolume.keySet()){
		if(transitlinkVolume.get(transitlinkid)==0) {
			error=0;
		}else {
			double currentVolume=this.transitLinks.get(timeBeanId).get(transitlinkid).getPassangerCount();
			double newVolume=transitlinkVolume.get(transitlinkid);
			error=Math.pow((currentVolume-newVolume),2);
			if(error/newVolume*100>tollerance) {

				sum+=1;
			}
			if(error<1) {
				linkBelow1++;
			}
		}
		if(error==Double.NaN||error==Double.NEGATIVE_INFINITY) {
			throw new IllegalArgumentException("Stop!!! There is something wrong!!!");
		}
		squareSum+=error;
	}
	if(squareSum==Double.NaN) {
		System.out.println("WAIT!!!!Problem!!!!!");
	}
	squareSum=Math.sqrt(squareSum);
	if(counter==1) {
		this.error.get(timeBeanId).clear();
	}
	this.error.get(timeBeanId).add(squareSum);
	logger.info("ERROR amount for "+timeBeanId+" = "+squareSum);
	//System.out.println("in timeBean Id "+timeBeanId+" No of link not converged = "+sum);
	
//	try {
//		//CNLSUEModel.writeData(timeBeanId+","+counter+","+squareSum+","+sum, this.fileLoc+"ErrorData"+timeBeanId+".csv");
//	} catch (IOException e) {
//		// TODO Auto-generated catch block
//		e.printStackTrace();
//	}
	
	if (squareSum<=1||sum==0||linkBelow1==linkVolume.size()+transitlinkVolume.size()){
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
			
		double carUtility=odPair.getExpectedMaximumCarUtility(params, anaParams, timeBeanId);
		double transitUtility=odPair.getExpectedMaximumTransitUtility(params, anaParams, timeBeanId);
		
		if(carUtility==Double.NEGATIVE_INFINITY||transitUtility==Double.POSITIVE_INFINITY||
				Math.exp(transitUtility*modeMiu)==Double.POSITIVE_INFINITY) {
			this.carDemand.get(timeBeanId).put(odPair.getODpairId(), 0.0);
			
		}else if(transitUtility==Double.NEGATIVE_INFINITY||carUtility==Double.POSITIVE_INFINITY
				||Math.exp(carUtility*modeMiu)==Double.POSITIVE_INFINITY) {
			this.carDemand.get(timeBeanId).put(odPair.getODpairId(), this.Demand.get(timeBeanId).get(odPair.getODpairId()));
		}else if(carUtility==Double.NEGATIVE_INFINITY && transitUtility==Double.NEGATIVE_INFINITY){
			this.carDemand.get(timeBeanId).put(odPair.getODpairId(), 0.);
		}else {
			double carProportion=Math.exp(carUtility*modeMiu)/(Math.exp(carUtility*modeMiu)+Math.exp(transitUtility*modeMiu));
			//System.out.println("Car Proportion = "+carProportion);
			Double cardemand=Math.exp(carUtility*modeMiu)/(Math.exp(carUtility*modeMiu)+Math.exp(transitUtility*modeMiu))*this.Demand.get(timeBeanId).get(odPair.getODpairId());
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
		this.routeProb.put(timeId, new ConcurrentHashMap<>());
		this.trRouteProb.put(timeId, new ConcurrentHashMap<>());
		this.expectedMaximumCarUtility.put(timeId, new ConcurrentHashMap<>());
		this.expectedMaximumTrUtility.put(timeId, new ConcurrentHashMap<>());
		for(Id<AnalyticalModelODpair> o:this.Demand.get(timeId).keySet()) {
			this.carDemand.get(timeId).put(o, this.Demand.get(timeId).get(o)*0.5);//This needs more attention
			
		}

	}
}

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
}
