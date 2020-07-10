package AnalyticalModel;

import java.util.ArrayList;
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
import org.matsim.vehicles.Vehicles;

import dynamicTransitRouter.fareCalculators.FareCalculator;
import transitCalculatorsWithFare.FareLink;
import ust.hk.praisehk.metamodelcalibration.analyticalModel.AnalyticalModelNetwork;
import ust.hk.praisehk.metamodelcalibration.analyticalModel.AnalyticalModelODpair;
import ust.hk.praisehk.metamodelcalibration.analyticalModel.AnalyticalModelRoute;
import ust.hk.praisehk.metamodelcalibration.analyticalModel.AnalyticalModelTransitRoute;
import ust.hk.praisehk.metamodelcalibration.analyticalModel.SUEModelOutput;
import ust.hk.praisehk.metamodelcalibration.analyticalModel.TransitLink;
import ust.hk.praisehk.metamodelcalibration.analyticalModelImpl.CNLLink;
import ust.hk.praisehk.metamodelcalibration.analyticalModelImpl.CNLNetwork;
import ust.hk.praisehk.metamodelcalibration.analyticalModelImpl.CNLODpairs;
import ust.hk.praisehk.metamodelcalibration.analyticalModelImpl.CNLSUEModel;
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
private Map<String,HashMap<Id<AnalyticalModelODpair>,Double>> Demand=new ConcurrentHashMap<>();//Holds ODpair based demand
private Map<String,HashMap<Id<AnalyticalModelODpair>,Double>> carDemand=new ConcurrentHashMap<>(); 
private CNLODpairs odPairs;
private Map<String,Map<Id<TransitLink>,TransitLink>> transitLinks=new ConcurrentHashMap<>();
	
private Population lastPopulation;

//This are needed for output generation 

protected Map<String,Map<Id<Link>,Double>> outputLinkTT=new ConcurrentHashMap<>();
protected Map<String,Map<Id<TransitLink>,Double>> outputTrLinkTT=new ConcurrentHashMap<>();
private Map<String,Map<Id<Link>,Double>> totalPtCapacityOnLink=new HashMap<>();
protected Map<String,Map<String,Double>>MTRCount=new ConcurrentHashMap<>();
//All the parameters name
//They are kept public to make it easily accessible as they are final they can not be modified

private boolean emptyMeasurements;



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
						((CNLLink)network.getLinks().get(linkId)).addLinkTransitVolume(vehicles.getVehicles().get(d.getVehicleId()).getType().getPcuEquivalents());
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
	if(!(this.Params.keySet()).containsAll(params.keySet())) {
		logger.error("The parameters key do not match with the default parameter keys. Invalid Parameter!! Did you send the wrong parameter format?");
		//throw new IllegalArgumentException("The parameters key do not match with the default parameter keys. Invalid Parameter!! Did you send the wrong parameter format?");
	}
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

protected HashMap<Id<Link>,Double> NetworkLoadingCarSingleOD(Id<AnalyticalModelODpair> ODpairId,String timeBeanId,double counter,LinkedHashMap<String,Double> params, LinkedHashMap<String, Double> anaParams){

	AnalyticalModelODpair odpair=this.odPairs.getODpairset().get(ODpairId);
	String subpopulation = odpair.getSubPopulation();
	
	List<AnalyticalModelRoute> routes=odpair.getRoutes();
	HashMap<Id<AnalyticalModelRoute>,Double> routeFlows=new HashMap<>();
	HashMap<Id<Link>,Double> linkFlows=new HashMap<>();
	
	
	//double totalUtility=0;
	
	//Calculating route utility for all car routes inside one OD pair.
	
	//HashMap<Id<AnalyticalModelRoute>,Double> oldUtility=new HashMap<>();
	HashMap<Id<AnalyticalModelRoute>,Double> utility=new HashMap<>();
	for(AnalyticalModelRoute r:routes){
		double u=0;
		
		if(counter>1) {
			u=r.calcRouteUtility(params, anaParams,this.getNetworks().get(timeBeanId),this.timeBeans.get(timeBeanId));
			u=u+Math.log(odpair.getAutoPathSize().get(r.getRouteId()));//adding the path size term
			utility.put(r.getRouteId(), u);
			//oldUtility.put(r.getRouteId(),this.getOdPairs().getODpairset().get(ODpairId).getRouteUtility(timeBeanId).get(r.getRouteId()));
		}else {
			u=0;
			utility.put(r.getRouteId(), u);
		}
		//oldUtility.put(r.getRouteId(),this.odPairs.getODpairset().get(ODpairId).getRouteUtility(timeBeanId).get(r.getRouteId()));
		odpair.updateRouteUtility(r.getRouteId(), u,timeBeanId);
		
		//This Check is to make sure the exp(utility) do not go to infinity.
		if(u>300||u<-300) {
			logger.error("utility is either too small or too large. Increase or decrease the link miu accordingly. The utility is "+u+" for route "+r.getRouteId());
			//throw new IllegalArgumentException("stop!!!");
		}
		//totalUtility+=Math.exp(u);
	}
//	if(routes.size()>1 && counter>2 ) {
//		//&& (error.get(timeBeanId).get((int)(counter-2))>error.get(timeBeanId).get((int)(counter-3)))
//		System.out.println("Testing!!!");
//		for(CNLRoute r:routes) {
//			double diff=(newUtility.get(r.getRouteId())-oldUtility.get(r.getRouteId()));
//			if(Math.pow(diff,2)>0.00002){
//				
//				System.out.println(diff);
//			}
//		}
//	}
	//If total utility is zero, then there should not be any route. For testing purpose, can be removed later 
//	if(totalUtility==0) {
//		logger.error("utility is zero. Please check.");
//		throw new IllegalArgumentException("Stop!!!!");
//	}
	
	
	//This is the route flow split
	for(AnalyticalModelRoute r:routes){
		double u=utility.get(r.getRouteId());
		double demand=this.getCarDemand().get(timeBeanId).get(ODpairId);
		String id=null;
		if(this.odMultiplierId==null) {
			id=CNLSUEModel.getODtoODMultiplierId(odpair.getODpairId().toString(),timeBeanId);
		}else {
			id=this.odMultiplierId.get(odpair.getODpairId()).get(timeBeanId);
		}
		if(params.containsKey(id)) {
			demand=demand*params.get(id);
		}
		double totalUtility=0;
		for(double d:utility.values()) {
			totalUtility+=Math.exp(d-u);
		}
		double flow=1/totalUtility*demand;
		//For testing purpose, can be removed later
		if(flow==Double.NaN||flow==Double.POSITIVE_INFINITY) {
			logger.error("The flow is NAN. This can happen for a number of reasons. Mostly is total utility of all the routes in a OD pair is zero");
			throw new IllegalArgumentException("Wait!!!!Error!!!!");
		}
		routeFlows.put(r.getRouteId(),flow);
		odpair.getRouteFlow().get(timeBeanId).put(r.getRouteId(), flow);	
	}
	for(Id<Link> linkId:getOdPairs().getODpairset().get(ODpairId).getLinkIncidence().keySet()){
		double linkflow=0;
		for(AnalyticalModelRoute r:getOdPairs().getODpairset().get(ODpairId).getLinkIncidence().get(linkId)){
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
	
	AnalyticalModelODpair odpair=this.od.getODpairset().get(ODpairId);
	List<AnalyticalModelTransitRoute> routes=odpair.getTrRoutes(timeBeanId);
	
	HashMap<Id<AnalyticalModelTransitRoute>,Double> routeFlows=new HashMap<>();
	HashMap<Id<TransitLink>,Double> linkFlows=new HashMap<>();
	
	HashMap<Id<AnalyticalModelTransitRoute>,Double> utility=new HashMap<>();
	
	if(routes!=null && routes.size()!=0) {
	for(AnalyticalModelTransitRoute r:routes){
		double u=0;
		if(counter>1) {
			u=r.calcRouteUtility(params, anaParams,
				this.getNetworks().get(timeBeanId),this.fareCalculator,null,this.timeBeans.get(timeBeanId));
			u+=Math.log(odpair.getTrPathSize().get(timeBeanId).get(r.getTrRouteId()));//adding the path size term
			
			if(u==Double.NaN) {
				logger.error("The flow is NAN. This can happen for a number of reasons. Mostly is total utility of all the routes in a OD pair is zero");
				throw new IllegalArgumentException("Utility is NAN!!!");
			}
		}else {
			u=0;
		}
		if(u>300) {
			logger.warn("STOP!!!Utility is too large >300");
		}
		odpair.updateTrRouteUtility(r.getTrRouteId(), u,timeBeanId);
		utility.put(r.getTrRouteId(), u);
		//totalUtility+=Math.exp(u);
	}
//	if(totalUtility==0) {
//		logger.warn("STopp!!!! Total utility in the OD pair is zero. This can happen if there is no transit route in that OD pair.");
//	}
	for(AnalyticalModelTransitRoute r:routes){
		double totalDemand=this.getDemand().get(timeBeanId).get(ODpairId);
		double carDemand=this.getCarDemand().get(timeBeanId).get(ODpairId);
		double q=(totalDemand-carDemand);
		String id=null;
		if(this.odMultiplierId==null) {
			id=CNLSUEModel.getODtoODMultiplierId(odpair.getODpairId().toString(),timeBeanId);
		}else {
			id=this.odMultiplierId.get(odpair.getODpairId()).get(timeBeanId);
		}
		if(params.containsKey(id)) {
			double d=params.get(id);
			q=q*d;
		}
		double u=utility.get(r.getTrRouteId());
		double totalUtility=0;
		for(double d:utility.values()) {
			totalUtility+=Math.exp(d-u);
		}
		
		double flow=q/totalUtility;
		if(Double.isNaN(flow)||flow==Double.POSITIVE_INFINITY||flow==Double.NEGATIVE_INFINITY) {
			logger.error("The flow is NAN. This can happen for a number of reasons. Mostly is total utility of all the routes in a OD pair is zero");
			throw new IllegalArgumentException("Error!!!!");
		}
		routeFlows.put(r.getTrRouteId(),flow);
		odpair.getTrRouteFlow().get(timeBeanId).put(r.getTrRouteId(), flow);		
	}

	}
	
	Set<Id<TransitLink>>linksets=getOdPairs().getODpairset().get(ODpairId).getTrLinkIncidence().keySet();
	for(Id<TransitLink> linkId:linksets){
		if(this.getTransitLinks().get(timeBeanId).containsKey(linkId)) {
		double linkflow=0;
		ArrayList<AnalyticalModelTransitRoute>incidence=getOdPairs().getODpairset().get(ODpairId).getTrLinkIncidence().get(linkId);
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


}
