package core;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;
import java.util.stream.Collectors;

import org.matsim.api.core.v01.Id;
import org.matsim.api.core.v01.network.Link;

import AnalyticalModel.ODDifferentiableSUEModel;
import optimizer.VariableDetails;
import population.TPUSB;
import ust.hk.praisehk.metamodelcalibration.analyticalModel.AnalyticalModelODpair;
import ust.hk.praisehk.metamodelcalibration.calibrator.ObjectiveCalculator;
import ust.hk.praisehk.metamodelcalibration.measurements.Measurement;
import ust.hk.praisehk.metamodelcalibration.measurements.MeasurementType;
import ust.hk.praisehk.metamodelcalibration.measurements.Measurements;

public class ODUtils {
	public static final String originaDestinationDemandVariableName = "ODDemand";
	public static final String originaDestinationSubPopSepcificDemandVariableName = "ODDemandSubPop";
	public static final String originaDestinationSubPopTimeSepcificDemandVariableName = "ODDemandSubPopTime";
	public static final String originDestinationTimeSpecificDemandVariableName = "ODDemandTime";
	public static final String OriginMultiplierVariableName = "originMultiplier";
	public static final String DestinationMultiplierVariableName = "destinationMultiplier";
	public static final String OriginDestinationMultiplierVariableName = "originDestinationMultipler";
	public static final String OriginDestinationSubPopMultiplierVariableName = "originDestinationSubPopMultipler";
	public static final String OriginMultiplierSubPopVaraibleName = "originSubPopMultiplier";
	public static final String DestinationMultiplierSubPopVaraibleName = "destinationSubPopMultiplier";
	public static final String originMultiplierTimeSpecificSubPopVariableName = "originSubTimeMultiplier";
	public static final String originMultiplierTimeSpecificVariableName = "originTimeMultiplier";
	public static final String destinationMultiplierTimeSpecificSubPopVariableName = "destinationSubTimeMultiplier";
	public static final String destinationMultiplierTimeSpecificVariableName = "destinationTimeMultiplier";
	public static final String origindestinationMultiplierTimeSpecificSubPopVariableName = "originDestinationSubTimeMultiplier";
	public static final String origindestinationMultiplierTimeSpecificVariableName = "originDestinationTimeMultiplier";
	
	public static final Set<String> keySets = new HashSet<>(Arrays.asList(
			originaDestinationDemandVariableName,
			originaDestinationSubPopSepcificDemandVariableName,
			originaDestinationSubPopTimeSepcificDemandVariableName,
			originDestinationTimeSpecificDemandVariableName,
			OriginMultiplierVariableName,
			DestinationMultiplierVariableName,
			OriginDestinationMultiplierVariableName,
			OriginDestinationSubPopMultiplierVariableName,
			OriginMultiplierSubPopVaraibleName,
			DestinationMultiplierSubPopVaraibleName,
			originMultiplierTimeSpecificSubPopVariableName,
			originMultiplierTimeSpecificVariableName,
			destinationMultiplierTimeSpecificSubPopVariableName,
			destinationMultiplierTimeSpecificVariableName,
			origindestinationMultiplierTimeSpecificSubPopVariableName,
			origindestinationMultiplierTimeSpecificVariableName));
	
	//TODO: take subpopulation as input
	public static String createODMultiplierVariableName(Id<AnalyticalModelODpair> odPairId,String subPopulationName, String type, String timeBeanId) {
		String[] part = odPairId.toString().split("_");
		String originId = part[0];
		String destinationId = part[1];
		String subPopulation = subPopulationName;
		switch (type) {
		case OriginMultiplierVariableName:
			return OriginMultiplierVariableName+"___"+originId;
		case DestinationMultiplierVariableName:
			return DestinationMultiplierVariableName+"___"+destinationId;
		case OriginDestinationMultiplierVariableName:
			return OriginDestinationMultiplierVariableName+"___"+originId+"___"+destinationId;
		case OriginDestinationSubPopMultiplierVariableName:
			return OriginDestinationSubPopMultiplierVariableName+"___"+originId+"___"+destinationId+"___"+subPopulation;
		case OriginMultiplierSubPopVaraibleName:
			return OriginMultiplierSubPopVaraibleName+"___"+originId+"___"+subPopulation;
		case DestinationMultiplierSubPopVaraibleName:
			return DestinationMultiplierSubPopVaraibleName+"___"+destinationId+"___"+subPopulation;
		case originMultiplierTimeSpecificSubPopVariableName:
			return originMultiplierTimeSpecificSubPopVariableName+"___"+originId+"___"+subPopulation+"___"+timeBeanId;
		case destinationMultiplierTimeSpecificSubPopVariableName:
			return destinationMultiplierTimeSpecificSubPopVariableName+"___"+destinationId+"___"+subPopulation+"___"+timeBeanId;
		case origindestinationMultiplierTimeSpecificSubPopVariableName:
			return origindestinationMultiplierTimeSpecificSubPopVariableName+"___"+originId+"___"+destinationId+"___"+subPopulation+"___"+timeBeanId;
		case originaDestinationDemandVariableName:
			return originaDestinationDemandVariableName+"___"+originId+"___"+destinationId;
		case originaDestinationSubPopSepcificDemandVariableName:
			return originaDestinationSubPopSepcificDemandVariableName+"___"+originId+"___"+destinationId+"___"+subPopulation;
		case originaDestinationSubPopTimeSepcificDemandVariableName:
			return originaDestinationSubPopTimeSepcificDemandVariableName+"___"+originId+"___"+destinationId+"___"+subPopulation+"___"+timeBeanId;
		case originDestinationTimeSpecificDemandVariableName:
			return originDestinationTimeSpecificDemandVariableName+"___"+originId+"___"+destinationId+"___"+timeBeanId;
		case originMultiplierTimeSpecificVariableName:
			return originMultiplierTimeSpecificVariableName+"___"+originId+"___"+timeBeanId;
		case destinationMultiplierTimeSpecificVariableName:
			return destinationMultiplierTimeSpecificVariableName+"___"+destinationId+"___"+timeBeanId;
		case origindestinationMultiplierTimeSpecificVariableName:
			return origindestinationMultiplierTimeSpecificVariableName+"___"+originId+"___"+destinationId+"___"+timeBeanId;
		default:
			throw new IllegalArgumentException("Input type: "+type+ "not recognized. Please use the static final string keys in the ODUtils class only.");	
		}
		
	}
	
	public static Map<String,Map<Id<AnalyticalModelODpair>,Double>> applyODPairMultiplier(Map<String,Map<Id<AnalyticalModelODpair>,Double>> inputODPairMap,LinkedHashMap<String,Double> variables, 
			Map<Id<AnalyticalModelODpair>,AnalyticalModelODpair>odPairs){
		
		Map<String,Map<Id<AnalyticalModelODpair>,Double>> outDemand = new HashMap<>();
		for(Entry<String, Map<Id<AnalyticalModelODpair>, Double>> timeDemand: inputODPairMap.entrySet()) {
			outDemand.put(timeDemand.getKey(), new HashMap<>());
			Map<Id<AnalyticalModelODpair>, Double> outTimeDemand = outDemand.get(timeDemand.getKey());
			
			for(Entry<Id<AnalyticalModelODpair>, Double> od:timeDemand.getValue().entrySet()) {
			//timeDemand.getValue().entrySet().parallelStream().forEach(od-> {
				Double m = 1.;
				outTimeDemand.put(od.getKey(), od.getValue());
				if((m = variables.get(createODMultiplierVariableName(od.getKey(),odPairs.get(od.getKey()).getSubPopulation(),OriginMultiplierVariableName,timeDemand.getKey())))!=null) {
					outTimeDemand.put(od.getKey(), outTimeDemand.get(od.getKey())*m);
				}
				
				if((m =variables.get(createODMultiplierVariableName(od.getKey(),odPairs.get(od.getKey()).getSubPopulation(),DestinationMultiplierVariableName,timeDemand.getKey())))!=null) {
					outTimeDemand.put(od.getKey(), outTimeDemand.get(od.getKey())*m);
				}
				if((m =variables.get(createODMultiplierVariableName(od.getKey(),odPairs.get(od.getKey()).getSubPopulation(),OriginDestinationMultiplierVariableName,timeDemand.getKey())))!=null) {
					outTimeDemand.put(od.getKey(), outTimeDemand.get(od.getKey())*m);
				}
				if((m =variables.get(createODMultiplierVariableName(od.getKey(),odPairs.get(od.getKey()).getSubPopulation(),OriginDestinationSubPopMultiplierVariableName,timeDemand.getKey())))!=null) {
					outTimeDemand.put(od.getKey(), outTimeDemand.get(od.getKey())*m);
				}
				if((m =variables.get(createODMultiplierVariableName(od.getKey(),odPairs.get(od.getKey()).getSubPopulation(),OriginMultiplierSubPopVaraibleName,timeDemand.getKey())))!=null) {
					outTimeDemand.put(od.getKey(), outTimeDemand.get(od.getKey())*m);
				}
				if((m =variables.get(createODMultiplierVariableName(od.getKey(),odPairs.get(od.getKey()).getSubPopulation(),DestinationMultiplierSubPopVaraibleName,timeDemand.getKey())))!=null) {
					outTimeDemand.put(od.getKey(), outTimeDemand.get(od.getKey())*m);
				}
				if((m =variables.get(createODMultiplierVariableName(od.getKey(),odPairs.get(od.getKey()).getSubPopulation(),originMultiplierTimeSpecificSubPopVariableName,timeDemand.getKey())))!=null) {
					outTimeDemand.put(od.getKey(), outTimeDemand.get(od.getKey())*m);
				}
				if((m =variables.get(createODMultiplierVariableName(od.getKey(),odPairs.get(od.getKey()).getSubPopulation(),destinationMultiplierTimeSpecificSubPopVariableName,timeDemand.getKey())))!=null) {
					outTimeDemand.put(od.getKey(), outTimeDemand.get(od.getKey())*m);
				}
				if((m =variables.get(createODMultiplierVariableName(od.getKey(),odPairs.get(od.getKey()).getSubPopulation(),origindestinationMultiplierTimeSpecificSubPopVariableName,timeDemand.getKey())))!=null) {
					outTimeDemand.put(od.getKey(), outTimeDemand.get(od.getKey())*m);
				}
				if((m =variables.get(createODMultiplierVariableName(od.getKey(),odPairs.get(od.getKey()).getSubPopulation(),originaDestinationDemandVariableName,timeDemand.getKey())))!=null) {
					outTimeDemand.put(od.getKey(), m);
				}
				if((m =variables.get(createODMultiplierVariableName(od.getKey(),odPairs.get(od.getKey()).getSubPopulation(),originaDestinationSubPopSepcificDemandVariableName,timeDemand.getKey())))!=null) {
					outTimeDemand.put(od.getKey(), m);
				}
				if((m =variables.get(createODMultiplierVariableName(od.getKey(),odPairs.get(od.getKey()).getSubPopulation(),originaDestinationSubPopTimeSepcificDemandVariableName,timeDemand.getKey())))!=null) {
					outTimeDemand.put(od.getKey(), m);
				}
				if((m =variables.get(createODMultiplierVariableName(od.getKey(),odPairs.get(od.getKey()).getSubPopulation(),originDestinationTimeSpecificDemandVariableName,timeDemand.getKey())))!=null) {
					outTimeDemand.put(od.getKey(), m);
				}
				if((m =variables.get(createODMultiplierVariableName(od.getKey(),odPairs.get(od.getKey()).getSubPopulation(),originMultiplierTimeSpecificVariableName,timeDemand.getKey())))!=null) {
					outTimeDemand.put(od.getKey(), outTimeDemand.get(od.getKey())*m);
				}
				if((m =variables.get(createODMultiplierVariableName(od.getKey(),odPairs.get(od.getKey()).getSubPopulation(),destinationMultiplierTimeSpecificVariableName,timeDemand.getKey())))!=null) {
					outTimeDemand.put(od.getKey(), outTimeDemand.get(od.getKey())*m);
				}
				if((m =variables.get(createODMultiplierVariableName(od.getKey(),odPairs.get(od.getKey()).getSubPopulation(),origindestinationMultiplierTimeSpecificVariableName,timeDemand.getKey())))!=null) {
					outTimeDemand.put(od.getKey(), outTimeDemand.get(od.getKey())*m);
				}
		//	});
		
			}
		}
		return outDemand;
	}
	
	/**
	 * Extract only the od related keys from a set of parameter keys
	 * @param inputVarKeys
	 * @return
	 */
	public static Set<String> extractODVarKeys(Set<String> inputVarKeys){
		Set<String> varKeys = new HashSet<>();
		for(String s:inputVarKeys) {
			for(String aKeys:keySets) {
				if(s.contains(aKeys)) {
					varKeys.add(s);
					break;
				}
			}
		}
		return varKeys;	
	}
	
	/**
	 * check if the var key belongs to the 
	 * multiply this with the demand and divide by the current variable value and then multiply with pr*pm
	 * @param odId
	 * @param timeBeanId
	 * @param varKey
	 * @return
	 */
	public static int ifMatch_1_else_0(Id<AnalyticalModelODpair> odId,String subPopulationName, String timeBeanId,String varKey) {
		if(createODMultiplierVariableName(odId,subPopulationName, varKey.split("___")[0],timeBeanId).equals(varKey)) {
			return 1;
		}
		return 0;
	}
	
	
	public static double calcODObjective(Measurements realMeasurements, Measurements modelMeasurements) {
		return ObjectiveCalculator.calcObjective(realMeasurements, modelMeasurements, ObjectiveCalculator.TypeMeasurementAndTimeSpecific)*0.5;
	}
	
	/**
	 * The only way to parallelize it is to make variable parallelized
	 * TODO: try that
	 * @param realMeasurements
	 * @param modelMeasurements
	 * @param model
	 * @return
	 */
	public static Map<String, Double> calcODObjectiveGradient(Measurements realMeasurements, Measurements modelMeasurements, ODDifferentiableSUEModel model) {
		Map<String,Double> outGrad = model.getGradientKeys().stream().collect(Collectors.toMap(kk->kk, kk->0.));
		realMeasurements.getMeasurements().values().forEach(m->{
			m.getVolumes().entrySet().forEach(timeId->{
				double delta = modelMeasurements.getMeasurements().get(m.getId()).getVolumes().get(timeId.getKey()) - timeId.getValue();
				MeasurementType type = m.getMeasurementType();
				Map<String,Double> grad = new HashMap<>();
				if(type.equals(MeasurementType.linkVolume)) {
					List<Id<Link>>links = (List<Id<Link>>)m.getAttribute(Measurement.linkListAttributeName);
					for(Id<Link>l:links) {
						for(Entry<String, Double> gd:model.getLinkGradient().get(timeId.getKey()).get(l).entrySet()){
							grad.compute(gd.getKey(), (k,v)->v==null?gd.getValue()*delta:v+gd.getValue()*delta);
						}
					}
					//grad = model.getLinkGradient().get(timeId.getKey()).get()
				}else if(type.equals(MeasurementType.fareLinkVolume)) {
					grad = model.getFareLinkGradient().get(timeId.getKey()).get(m.getAttribute(Measurement.FareLinkAttributeName));
					for(String k:grad.keySet())grad.put(k,grad.get(k)*delta);
				}
				for(String var:model.getGradientKeys()) {
					outGrad.put(var, outGrad.get(var)+grad.get(var));
				}
			});
		});
		return outGrad;
	}
	
	/**
	 * The only way to parallelize it is to make variable parallelized
	 * TODO: try that
	 * @param realMeasurements
	 * @param modelMeasurements
	 * @param model
	 * @return
	 */
	public static Map<String, Double> calcODObjectiveGradient(Measurements realMeasurements, Map<String,VariableDetails> variables, ODDifferentiableSUEModel model) {
		LinkedHashMap<String,Double> params = new LinkedHashMap<>();
		variables.values().stream().forEach(v->params.put(v.getVariableName(), v.getCurrentValue()));
		Measurements modelMeasurements = model.perFormSUE(params, realMeasurements);
		Map<String,Double> outGrad = model.getGradientKeys().stream().collect(Collectors.toMap(kk->kk, kk->0.));
		realMeasurements.getMeasurements().values().forEach(m->{
			m.getVolumes().entrySet().forEach(timeId->{
				double delta = modelMeasurements.getMeasurements().get(m.getId()).getVolumes().get(timeId.getKey()) - timeId.getValue();
				MeasurementType type = m.getMeasurementType();
				Map<String,Double> grad = new HashMap<>();
				if(type.equals(MeasurementType.linkVolume)) {
					List<Id<Link>>links = (List<Id<Link>>)m.getAttribute(Measurement.linkListAttributeName);
					for(Id<Link>l:links) {
						for(Entry<String, Double> gd:model.getLinkGradient().get(timeId.getKey()).get(l).entrySet()){
							grad.compute(gd.getKey(), (k,v)->v==null?gd.getValue()*delta:v+gd.getValue()*delta);
						}
					}
					//grad = model.getLinkGradient().get(timeId.getKey()).get()
				}else if(type.equals(MeasurementType.fareLinkVolume)) {
					grad = model.getFareLinkGradient().get(timeId.getKey()).get(m.getAttribute(Measurement.FareLinkAttributeName));
					for(String k:grad.keySet())grad.put(k,grad.get(k)*delta);
				}
				for(String var:model.getGradientKeys()) {
					outGrad.put(var, outGrad.get(var)+grad.get(var));
				}
			});
		});
		return outGrad;
	}
}
