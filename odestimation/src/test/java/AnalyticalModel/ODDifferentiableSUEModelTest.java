package AnalyticalModel;

import static org.junit.jupiter.api.Assertions.*;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;
import java.util.stream.Collectors;

import javax.xml.parsers.ParserConfigurationException;
import javax.xml.parsers.SAXParser;
import javax.xml.parsers.SAXParserFactory;

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealVector;
import org.junit.jupiter.api.Test;
import org.matsim.api.core.v01.Id;
import org.matsim.api.core.v01.Scenario;
import org.matsim.api.core.v01.network.Link;
import org.matsim.api.core.v01.network.Network;
import org.matsim.api.core.v01.population.Activity;
import org.matsim.api.core.v01.population.Person;
import org.matsim.api.core.v01.population.PlanElement;
import org.matsim.core.config.Config;
import org.matsim.core.config.ConfigUtils;
import org.matsim.core.config.groups.PlanCalcScoreConfigGroup.ActivityParams;
import org.matsim.core.config.groups.PlanCalcScoreConfigGroup.TypicalDurationScoreComputation;
import org.matsim.core.network.NetworkUtils;
import org.matsim.core.population.PopulationUtils;
import org.matsim.core.scenario.MutableScenario;
import org.matsim.core.scenario.ScenarioUtils;
import org.matsim.core.utils.collections.Tuple;
import org.matsim.utils.objectattributes.ObjectAttributes;
import org.matsim.utils.objectattributes.ObjectAttributesXmlReader;
import org.matsim.vehicles.Vehicle;
import org.matsim.vehicles.VehicleType;
import org.matsim.vehicles.VehicleUtils;
import org.matsim.vehicles.VehicleWriterV1;
import org.matsim.vehicles.Vehicles;
import org.xml.sax.SAXException;

import com.google.common.collect.Maps;

import core.MapToArray;
import core.ODUtils;
import createPTGTFS.FareCalculatorPTGTFS;
import dynamicTransitRouter.fareCalculators.FareCalculator;
import dynamicTransitRouter.fareCalculators.LRFareCalculator;
import dynamicTransitRouter.fareCalculators.MTRFareCalculator;
import dynamicTransitRouter.fareCalculators.UniformFareCalculator;
import dynamicTransitRouter.fareCalculators.ZonalFareXMLParserV2;
import optimizer.Adam;
import optimizer.GD;
import optimizer.Optimizer;
import optimizer.VariableDetails;
import ust.hk.praisehk.metamodelcalibration.calibrator.ObjectiveCalculator;
import ust.hk.praisehk.metamodelcalibration.measurements.Measurement;
import ust.hk.praisehk.metamodelcalibration.measurements.MeasurementType;
import ust.hk.praisehk.metamodelcalibration.measurements.Measurements;
import ust.hk.praisehk.metamodelcalibration.measurements.MeasurementsReader;

class ODDifferentiableSUEModelTest {

	//public static void main(String[] args) {
	public static void main(String[] args) {
		Measurements originalMeasurements = new MeasurementsReader().readMeasurements("fullHk/ATCMeasurementsPeakFuLLHK.xml");
		Network net = NetworkUtils.readNetwork("fullHk/output_network.xml.gz");
		for(Measurement m:new ArrayList<>(originalMeasurements.getMeasurementsByType().get(MeasurementType.linkVolume))) {
			List<Id<Link>> linkList = (List<Id<Link>>)m.getAttribute(Measurement.linkListAttributeName);
			for(Id<Link>lId:linkList) {
				if(!net.getLinks().containsKey(lId)) {
					originalMeasurements.removeMeasurement(m.getId());
					break;
				}
			}
		}
		Map<String,Measurements> timeSplitMeasurements=timeSplitMeasurements(originalMeasurements);
		Config config = ConfigUtils.createConfig();
		ConfigUtils.loadConfig("fullHk/output_config.xml");
		config.transit().setTransitScheduleFile("fullHk/output_transitSchedule.xml.gz");
		config.transit().setVehiclesFile("fullHk/output_transitVehicles.xml.gz");
		//config.vehicles().setVehiclesFile("fullHk/output_vehicles.xml.gz");
		config.vehicles().setVehiclesFile("fullHk/vehicles_reduced.xml");
		config.plans().setInputFile("fullHk/population_reduced.xml");
		config.network().setInputFile("fullHk/output_network.xml.gz");
		Scenario scenario = ScenarioUtils.loadScenario(config);
		//Vehicles vehicles = VehicleUtils.createVehiclesContainer();
		
		
		ObjectAttributes obj = new ObjectAttributes();
		new ObjectAttributesXmlReader(obj).readFile("fullHK/output_PersonAttributes.xml.gz");
		//for(Entry<Id<VehicleType>, VehicleType> vt:scenario.getVehicles().getVehicleTypes().entrySet())vehicles.addVehicleType(vt.getValue());
		for(Person person: scenario.getPopulation().getPersons().values()) {
			String subPop = (String) obj.getAttribute(person.getId().toString(), "SUBPOP_ATTRIB_NAME");
			PopulationUtils.putSubpopulation(person, subPop);
			//vehicles.addVehicle(scenario.getVehicles().getVehicles().get(Id.createVehicleId(person.getId().toString())));
		}
		
		//new VehicleWriterV1(vehicles).writeFile("fullHk/vehicles_reduced.xml");
		//vehicles = null;
		obj = null;
		Map<String,FareCalculator>fareCalculators = new HashMap<>();
		SAXParser saxParser;
		ZonalFareXMLParserV2 busFareGetter = new ZonalFareXMLParserV2(scenario.getTransitSchedule());
		try {
			saxParser = SAXParserFactory.newInstance().newSAXParser();
			saxParser.parse("fullHk/busFare.xml", busFareGetter);
			
		} catch (ParserConfigurationException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		} catch (SAXException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		try {
			fareCalculators.put("train", new MTRFareCalculator("fare/mtr_lines_fares.csv",scenario.getTransitSchedule()));
			fareCalculators.put("bus", FareCalculatorPTGTFS.loadFareCalculatorPTGTFS("fare/busFareGTFS.json"));
			fareCalculators.put("minibus", busFareGetter.get());
			fareCalculators.put("LR", new LRFareCalculator("fare/light_rail_fares.csv"));
			fareCalculators.put("ferry",FareCalculatorPTGTFS.loadFareCalculatorPTGTFS("fare/ferryFareGTFS.json"));
			fareCalculators.put("tram", new UniformFareCalculator(2.6));
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		Network odNetwork=NetworkUtils.readNetwork("fullHk/odNetwork.xml");
		for(Entry<String,Tuple<Double,Double>>timeBean:originalMeasurements.getTimeBean().entrySet()) {
			//if(!timeBean.getKey().equals("18"))continue;
			Map<String,Tuple<Double,Double>> singleTimeBean = new HashMap<>();
			singleTimeBean.put(timeBean.getKey(),timeBean.getValue());
			ODDifferentiableSUEModel model = new ODDifferentiableSUEModel(singleTimeBean, config);
			model.generateRoutesAndOD(scenario.getPopulation(), scenario.getNetwork(), odNetwork, scenario.getTransitSchedule(), scenario, fareCalculators);
			Set<String> uniqueVars = new HashSet<>();
			
			for(String timeId:originalMeasurements.getTimeBean().keySet()) {
				model.getOdPairs().getODpairset().entrySet().forEach(k->{
					uniqueVars.add(ODUtils.createODMultiplierVariableName(k.getKey(),k.getValue().getSubPopulation(), ODUtils.OriginMultiplierVariableName, timeId));
					uniqueVars.add(ODUtils.createODMultiplierVariableName(k.getKey(),k.getValue().getSubPopulation(), ODUtils.DestinationMultiplierVariableName, timeId));
				});
			}
			Map<String,VariableDetails> Param = new HashMap<>();
			for(String k:uniqueVars)Param.put(k, new VariableDetails(k, new Tuple<Double,Double>(0.1,8.), Math.sqrt(2.0)));
			Optimizer adam = new Adam("odOptim",Param,0.1,0.9,.999,10e-6);
			//Optimizer gd = new GD("odOptim",Param,0.00005,1000);
			for(int counter = 0;counter<100;counter++) {
				//System.out.println("GB: " + (double) (Runtime.getRuntime().totalMemory() - Runtime.getRuntime().freeMemory()) / (1024*1024*1024));
				long t = System.currentTimeMillis();
				singleTimeBean = new HashMap<>();
				singleTimeBean.put(timeBean.getKey(),timeBean.getValue());
				model = new ODDifferentiableSUEModel(singleTimeBean, config);
				model.generateRoutesAndOD(scenario.getPopulation(), scenario.getNetwork(), odNetwork, scenario.getTransitSchedule(), scenario, fareCalculators);
				LinkedHashMap<String,Double> params = new LinkedHashMap<>();
				Param.values().stream().forEach(v->params.put(v.getVariableName(), v.getCurrentValue()));
				//params = new LinkedHashMap<>(readParams("seperateODMultiplier/gradAndParam_18_0.csv"));
//				List<Double> v = new ArrayList<>(params.values());
//				double[] ppp = new double[params.size()];
//				for(int i = 0; i<ppp.length; i++)ppp[i] = v.get(i);
//				RealVector pp = MatrixUtils.createRealVector(ppp);
//				System.out.println(pp.isInfinite()||pp.isNaN());
				Measurements modelMeasurements = model.perFormSUE(params, timeSplitMeasurements.get(timeBean.getKey()));
				Map<String,Double> grad = ODUtils.calcODObjectiveGradient(timeSplitMeasurements.get(timeBean.getKey()), modelMeasurements, model);
				writeMeasurementsComparison(timeSplitMeasurements.get(timeBean.getKey()),modelMeasurements,counter,"seperateODMultiplier",timeBean.getKey());
				Param = adam.takeStep(grad);
				//Param = gd.takeStep(grad);
				double objective = ObjectiveCalculator.calcObjective(timeSplitMeasurements.get(timeBean.getKey()), modelMeasurements, ObjectiveCalculator.TypeMeasurementAndTimeSpecific);
				System.out.println("Finished iteration "+counter);
				System.out.println("Objective = "+objective);
				final Map<String,VariableDetails> p = new LinkedHashMap<String,VariableDetails>(Param);
				Map<String,Double> paramValues = Param.keySet().stream().collect(Collectors.toMap(k->k, k->p.get(k).getCurrentValue()));
				MapToArray<String> m2a = new MapToArray<String>("writerM2A",grad.keySet());
				double gradNorm = m2a.getRealVector(grad).getNorm();
				dumpData("seperateODMultiplier",counter,timeBean.getKey(),objective,paramValues,grad,gradNorm);
				System.out.println("Used Memory in GB: " + (double) (Runtime.getRuntime().totalMemory() - Runtime.getRuntime().freeMemory()) / (1024*1024*1024));
				System.out.println("Time Required for iteratio "+counter+" = "+(System.currentTimeMillis()-t)/1000+" seconds.");
				if(gradNorm<10)break;
				
			}
		}
	}

	public static Map<String,Double> readParams(String fileLoc){
		Map<String,Double> param = new HashMap<>();
		try {
			BufferedReader bf  = new BufferedReader(new FileReader(new File(fileLoc)));
			bf.readLine();
			String line = null;
			while((line = bf.readLine())!=null) {
				String[] part = line.split(",");
				param.put(part[0], Double.parseDouble(part[1]));
			}
			bf.close();
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return param;
	}
	
	private static Map<String,Measurements> timeSplitMeasurements(Measurements m){
		Map<String,Measurements> mOuts=new HashMap<>();
		for(String timeId:m.getTimeBean().keySet()) {
			Map<String,Tuple<Double,Double>> singleBeanTimeBean=new HashMap<>();
			singleBeanTimeBean.put(timeId, m.getTimeBean().get(timeId));
			Measurements mt=Measurements.createMeasurements(singleBeanTimeBean);
			mOuts.put(timeId, mt);
			for(Entry<Id<Measurement>, Measurement> d:m.getMeasurements().entrySet()) {
				if(d.getValue().getVolumes().containsKey(timeId)) {
					mt.createAnadAddMeasurement(d.getKey().toString(), d.getValue().getMeasurementType());
					for(Entry<String, Object> attribue:d.getValue().getAttributes().entrySet()) {
						mt.getMeasurements().get(d.getKey()).setAttribute(attribue.getKey(), attribue.getValue());
					}
					mt.getMeasurements().get(d.getKey()).putVolume(timeId, d.getValue().getVolumes().get(timeId));
				}
			}
		}
		
		return mOuts;
	}
	
	public static void dumpData(String folderLoc, int counter, String timeBean, double objective, Map<String,Double> param,Map<String,Double> grad,double gradNorm) {
		File file = new File(folderLoc);
		if(!file.exists())file.mkdir();
		String paramAndGradFileName = folderLoc+"/gradAndParam_"+timeBean+"_"+counter+".csv";
		MapToArray<String> m2a = new MapToArray<String>("writerM2A",param.keySet());
		Map<String,double[]> mapToWrite = new HashMap<>();
		mapToWrite.put("Variables",m2a.getMatrix(param));
		mapToWrite.put("Gradient",m2a.getMatrix(grad));
		
		m2a.writeCSV(mapToWrite, paramAndGradFileName);
		String iterLogerFileName = folderLoc+"/iterLogger"+timeBean+".csv";
		try {
			FileWriter fw = new FileWriter(new File(iterLogerFileName),true);
			if(counter == 0)fw.append("Iterantion,timeId,Objective,GradNorm\n");
			fw.append(counter+","+timeBean+","+objective+","+gradNorm+"\n");
			fw.flush();
			fw.close();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	 
	public static void writeMeasurementsComparison(Measurements realMeasurements, Measurements modelMeasurements, int counter, String fileLoc, String timeId) {
		fileLoc = fileLoc+"/measurementsComparison"+timeId+"_"+counter+".csv";
		try {
			FileWriter fw = new FileWriter(new File(fileLoc));
			fw.append("MeasurementId,tiemBeanId,realCount,modelCount,apa,geh\n");
			fw.flush();
			for(Entry<Id<Measurement>, Measurement> m:realMeasurements.getMeasurements().entrySet()) {
				for(Entry<String, Double> v:m.getValue().getVolumes().entrySet()) {
					Measurement mModel = modelMeasurements.getMeasurements().get(m.getKey());
					if(mModel!=null && mModel.getVolumes().containsKey(v.getKey())) {
						double apa = Math.abs(v.getValue()-mModel.getVolumes().get(v.getKey()))/v.getValue()*100;
						double geh = Math.sqrt(2*Math.pow(v.getValue()-mModel.getVolumes().get(v.getKey()),2)/(v.getValue()+mModel.getVolumes().get(v.getKey())));
						fw.append(m.getKey().toString()+","+v.getKey()+","+v.getValue()+","+mModel.getVolumes().get(v.getKey())+","+apa+","+geh+"\n");
						fw.flush();
					}
				}
			}
			fw.close();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}