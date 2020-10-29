package AnalyticalModel;

import static org.junit.jupiter.api.Assertions.*;

import java.io.IOException;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;
import java.util.stream.Collectors;

import javax.xml.parsers.ParserConfigurationException;
import javax.xml.parsers.SAXParser;
import javax.xml.parsers.SAXParserFactory;

import org.junit.jupiter.api.Test;
import org.matsim.api.core.v01.Scenario;
import org.matsim.api.core.v01.network.Network;
import org.matsim.core.config.Config;
import org.matsim.core.config.ConfigUtils;
import org.matsim.core.network.NetworkUtils;
import org.matsim.core.scenario.ScenarioUtils;
import org.matsim.core.utils.collections.Tuple;
import org.xml.sax.SAXException;

import core.ODUtils;
import createPTGTFS.FareCalculatorPTGTFS;
import dynamicTransitRouter.fareCalculators.FareCalculator;
import dynamicTransitRouter.fareCalculators.LRFareCalculator;
import dynamicTransitRouter.fareCalculators.MTRFareCalculator;
import dynamicTransitRouter.fareCalculators.UniformFareCalculator;
import dynamicTransitRouter.fareCalculators.ZonalFareXMLParserV2;
import optimizer.Adam;
import optimizer.Optimizer;
import optimizer.VariableDetails;
import ust.hk.praisehk.metamodelcalibration.calibrator.ObjectiveCalculator;
import ust.hk.praisehk.metamodelcalibration.measurements.Measurements;
import ust.hk.praisehk.metamodelcalibration.measurements.MeasurementsReader;

class ODDifferentiableSUEModelTest {

	@Test
	void testODDifferentiableSUEModel() {
		Measurements originalMeasurements = new MeasurementsReader().readMeasurements("fullHk/ATCMeasurementsPeakFuLLHK.xml");
		Config config = ConfigUtils.createConfig();
		ConfigUtils.loadConfig("new Data/cal/output_config.xml");
		config.transit().setTransitScheduleFile("fullHk/output_transitSchedule.xml.gz");
		config.transit().setVehiclesFile("fullHk/output_transitVehicles.xml.gz");
		config.vehicles().setVehiclesFile("fullHk/output_vehicles.xml.gz");
		config.plans().setInputFile("fullHk/population_reduced.xml");
		config.network().setInputFile("fullHk/output_network.xml.gz");
		Scenario scenario = ScenarioUtils.loadScenario(config);
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
		Map<String,Tuple<Double,Double>> timeBean = new HashMap<>();
		for(int i = 0; i<24 ;i++) {
			timeBean.put(Integer.toString(i+1), new Tuple<>(i*3600.,(i+1)*3600.));
		}
		Network odNetwork=NetworkUtils.readNetwork("new Data/data/odNetwork.xml");
		ODDifferentiableSUEModel model = new ODDifferentiableSUEModel(timeBean, config);
		model.generateRoutesAndOD(scenario.getPopulation(), scenario.getNetwork(), odNetwork, scenario.getTransitSchedule(), scenario, fareCalculators);
		Set<String> uniqueVars = new HashSet<>();
		
		for(String timeId:originalMeasurements.getTimeBean().keySet()) {
			model.getOdPairs().getODpairset().keySet().forEach(k->{
				uniqueVars.add(ODUtils.createODMultiplierVariableName(k,ODUtils.origindestinationMultiplierTimeSpecificVariableName, timeId));
			});
		}
		Map<String,VariableDetails> Param = new HashMap<>();
		for(String k:uniqueVars)Param.put(k, new VariableDetails(k, new Tuple<Double,Double>(0.1,8.), 2.0));
		Optimizer adam = new Adam("odOptim",Param,0.01,0.9,.999,10e-8);
		for(int counter = 0;counter<100;counter++) {
			System.out.println("KB: " + (double) (Runtime.getRuntime().totalMemory() - Runtime.getRuntime().freeMemory()) / 1024);
			LinkedHashMap<String,Double> params = new LinkedHashMap<>();
			Param.values().stream().forEach(v->params.put(v.getVariableName(), v.getCurrentValue()));
			Measurements modelMeasurements = model.perFormSUE(params, originalMeasurements);
			Map<String,Double> grad = ODUtils.calcODObjectiveGradient(originalMeasurements, modelMeasurements, model);
			Param = adam.takeStep(grad);
			ObjectiveCalculator.calcObjective(originalMeasurements, modelMeasurements, ObjectiveCalculator.TypeMeasurementAndTimeSpecific);
			System.out.println("KB: " + (double) (Runtime.getRuntime().totalMemory() - Runtime.getRuntime().freeMemory()) / 1024);
		}
		fail("Not yet implemented");
	}

	@Test
	void testGenerateRoutesAndOD() {
		fail("Not yet implemented");
	}

	@Test
	void testPerFormSUELinkedHashMapOfStringDoubleMeasurements() {
		fail("Not yet implemented");
	}

}
