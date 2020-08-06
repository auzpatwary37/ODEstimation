package AnalyticalModel;

import static org.junit.jupiter.api.Assertions.*;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

import javax.xml.parsers.ParserConfigurationException;
import javax.xml.parsers.SAXParser;
import javax.xml.parsers.SAXParserFactory;

import org.junit.jupiter.api.Test;
import org.matsim.api.core.v01.Scenario;
import org.matsim.core.config.Config;
import org.matsim.core.config.ConfigUtils;
import org.matsim.core.scenario.ScenarioUtils;
import org.xml.sax.SAXException;

import createPTGTFS.FareCalculatorPTGTFS;
import dynamicTransitRouter.fareCalculators.FareCalculator;
import dynamicTransitRouter.fareCalculators.LRFareCalculator;
import dynamicTransitRouter.fareCalculators.MTRFareCalculator;
import dynamicTransitRouter.fareCalculators.UniformFareCalculator;
import dynamicTransitRouter.fareCalculators.ZonalFareXMLParserV2;
import ust.hk.praisehk.metamodelcalibration.measurements.Measurements;
import ust.hk.praisehk.metamodelcalibration.measurements.MeasurementsReader;

class ODDifferentiableSUEModelTest {

	@Test
	void testODDifferentiableSUEModel() {
		Measurements originalMeasurements = new MeasurementsReader().readMeasurements("new Data/cal/ATCMeasurementsPeakHKI_3_27.xml");
		Config config = ConfigUtils.createConfig();
		ConfigUtils.loadConfig("new Data/cal/output_config.xml");
		config.transit().setTransitScheduleFile("new Data/cal/output_transitSchedule.xml.gz");
		config.transit().setVehiclesFile("new Data/cal/output_transitVehicles.xml.gz");
		config.vehicles().setVehiclesFile("new Data/cal/output_vehicles.xml.gz");
		config.plans().setInputFile("new Data/cal/output_plans.xml.gz");
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
			fareCalculators.put("ferry",FareCalculatorPTGTFS.loadFareCalculatorPTGTFS("fullHK/fare/ferryFareGTFS.json"));
			fareCalculators.put("tram", new UniformFareCalculator(2.6));
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		ODDifferentiableSUEModel model = new ODDifferentiableSUEModel(null, config);
		
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
