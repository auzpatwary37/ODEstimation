package core;

import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Map.Entry;

import org.matsim.api.core.v01.Id;

import population.TPUSB;
import ust.hk.praisehk.metamodelcalibration.analyticalModel.AnalyticalModelODpair;

public class ODUtils {
	public static final String OriginMultiplierVariableName = "originMultiplier";
	public static final String DestinationMultiplierVariableName = "destinationMultiplier";
	public static final String OriginDestinationMultiplierVariableName = "originDestinationMultipler";
	public static final String OriginDestinationSubPopMultiplierVariableName = "originDestinationSubPopMultipler";
	public static final String OriginMultiplierSubPopVaraibleName = "originSubPopMultiplier";
	public static final String DestinationMultiplierSubPopVaraibleName = "destinationSubPopMultiplier";
	
	
	public static String createODMultiplierVariableName(Id<AnalyticalModelODpair> odPairId,String type) {
		String[] part = odPairId.toString().split("_");
		String originId = part[0];
		String destinationId = part[1];
		String subPopulation = "";
		if(part.length==3) subPopulation = part[2];
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
		default:
			throw new IllegalArgumentException("Input type: "+type+ "not recognized. Please use the static final string keys in the ODUtils class only.");	
		}
		
	}
	
	public static Map<String,Map<Id<AnalyticalModelODpair>,Double>> applyODPairMultiplier(Map<String,Map<Id<AnalyticalModelODpair>,Double>> inputODPairMap,LinkedHashMap<String,Double> variables){
		
		Map<String,Map<Id<AnalyticalModelODpair>,Double>> outDemand = new HashMap<>();
		for(Entry<String, Map<Id<AnalyticalModelODpair>, Double>> timeDemand: inputODPairMap.entrySet()) {
			outDemand.put(timeDemand.getKey(), new HashMap<>());
			Map<Id<AnalyticalModelODpair>, Double> outTimeDemand = outDemand.get(timeDemand.getKey());
			for(Entry<Id<AnalyticalModelODpair>, Double> od: timeDemand.getValue().entrySet()) {
				Double m = 1.;
				outTimeDemand.put(od.getKey(), od.getValue());
				if((m = variables.get(createODMultiplierVariableName(od.getKey(),OriginMultiplierVariableName)))!=null) {
					outTimeDemand.put(od.getKey(), outTimeDemand.get(od.getKey())*m);
				}
				
				if((m =variables.get(createODMultiplierVariableName(od.getKey(),DestinationMultiplierVariableName)))!=null) {
					outTimeDemand.put(od.getKey(), outTimeDemand.get(od.getKey())*m);
				}
				if((m =variables.get(createODMultiplierVariableName(od.getKey(),OriginDestinationMultiplierVariableName)))!=null) {
					outTimeDemand.put(od.getKey(), outTimeDemand.get(od.getKey())*m);
				}
				if((m =variables.get(createODMultiplierVariableName(od.getKey(),OriginDestinationSubPopMultiplierVariableName)))!=null) {
					outTimeDemand.put(od.getKey(), outTimeDemand.get(od.getKey())*m);
				}
				if((m =variables.get(createODMultiplierVariableName(od.getKey(),OriginMultiplierSubPopVaraibleName)))!=null) {
					outTimeDemand.put(od.getKey(), outTimeDemand.get(od.getKey())*m);
				}
				if((m =variables.get(createODMultiplierVariableName(od.getKey(),DestinationMultiplierSubPopVaraibleName)))!=null) {
					outTimeDemand.put(od.getKey(), outTimeDemand.get(od.getKey())*m);
				}
				
			}
		}
		return outDemand;
	}
	

}
