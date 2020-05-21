import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

public class search {
	public class cities{
		Double lon;
		String city_name;
		Double lat;
		Double dist;
		Map<String,Double> adj_dist=new HashMap<>();
		List<String> adj_cities= new LinkedList<>();
		boolean visited=false;
		
		public void adj_cities(String name, Double distance_diff) {
			adj_dist.put(name,distance_diff);
			adj_cities.add(name);
		}
	}
}
