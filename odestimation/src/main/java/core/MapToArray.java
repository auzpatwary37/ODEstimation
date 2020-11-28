package core;

import java.util.List;
import java.util.ArrayList;
import java.util.Map;
import java.util.HashMap;
import java.util.Set;

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealVector;

public class MapToArray<T>{
private final List<T>keySet;
private final String id;

public MapToArray(String id, Map<T,Double> inputMap) {
	this.id = id;
	this.keySet = new ArrayList<>(inputMap.keySet());
}

public MapToArray(String id, Set<T> inputSet) {
	this.id = id;
	this.keySet = new ArrayList<>(inputSet);
}

public MapToArray(String id, List<T> inputList) {
	this.id = id;
	this.keySet = new ArrayList<>(inputList);
}

public Map<T,Double> getMap(double[] matrix){
	Map<T,Double> out = new HashMap<>();
	for(int i = 0;i<this.keySet.size();i++) {
		out.put(this.keySet.get(i), matrix[i]);
	}
	return out;
}

public Map<T,Double> extractMap(double[] matrix, Set<T> key){
	Map<T,Double> out = new HashMap<>();
	for(int i = 0;i<this.keySet.size();i++) {
		if(key.contains(this.keySet.get(i)))out.put(this.keySet.get(i), matrix[i]);
	}
	return out;
}

public double[] getMatrix(Map<T,Double> map) {
	double[] out = new double[keySet.size()];
	for(int i = 0;i<this.keySet.size();i++) {
		out[i] = map.get(this.keySet.get(i));
	}
	return out;
}

public List<T> getKeySet() {
	return keySet;
}

public String getId() {
	return id;
}

public RealVector getRealVector(Map<T,Double> map) {
	return MatrixUtils.createRealVector(this.getMatrix(map));
}

}
