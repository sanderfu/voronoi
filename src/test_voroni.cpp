#define JC_VORONOI_IMPLEMENTATION
#define JCV_REAL_TYPE double
#define JCV_ATAN2 atan2
#define JCV_FLT_MAX 1.7976931348623157E+308

#include "voroni/jc_voronoi.h"
#include "iostream"
#include <fstream>
#include "ros/ros.h"
#include "ros/package.h"
#include "boost/filesystem.hpp"
#include "gdal/ogrsf_frmts.h"
#include "usv_mission_planner/quadtree.h"

#include "usv_map/map_service.h"

// A hash function used to hash a pair of any kind
struct hash_pair
{
    template <class T1, class T2>
    std::size_t operator() (const std::pair<T1, T2> &pair) const {
        return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
    }
};
 

int main(int argc, char** argv){
    ros::init(argc,argv,"test_voroni");
    std::string path = ros::package::getPath("voroni");
    path.append("/data/debug/");

    std::string edges_path = path+"edges.csv";
    std::string points_path = path+"points.csv";
    
    if(!boost::filesystem::exists(path)){
        boost::filesystem::create_directory(path);
    }

    std::ofstream edge_file(edges_path);
    edge_file << "x_from,y_from,x_to,y_to\n";

    std::ofstream points_file(points_path);
    points_file << "x,y\n";

    std::string db_path_ = ros::package::getPath("voroni");
    db_path_.append("/data/test_map/check_db.sqlite");
    GDALAllRegister();
    GDALDataset* ds = (GDALDataset*) GDALOpenEx(db_path_.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL);
    OGRLayer* comparison_layer = ds->GetLayerByName("collision_dissolved");

    OGRFeature* feat;
    double total_area = 0;

    std::vector<jcv_point> points_vec;
    OGRPoint* point;
    jcv_point new_point;
    OGRLinearRing* ring;

    comparison_layer->ResetReading();
    while((feat = comparison_layer->GetNextFeature()) != NULL){
        ring = feat->GetGeometryRef()->toPolygon()->getExteriorRing();
        int num_points = ring->getNumPoints();
        for(int i=0; i<num_points-1;i++){
            new_point.x = ring->getX(i);
            new_point.y = ring->getY(i);
            points_vec.push_back(new_point);
        }
        OGRFeature::DestroyFeature(feat);
    }
    std::cout << "Points: " << points_vec.size() << std::endl;

    //Load quadtree for collision check
    OGRPoint point_lower;
    const char* wkt_lower = "POINT(-73.98275 40.50820)";
    OGRPoint point_upper;
    const char* wkt_upper = "POINT(-73.90424 40.58268)";
    point_lower.importFromWkt(&wkt_lower);
    point_upper.importFromWkt(&wkt_upper);

    ros::NodeHandle nh("~testQT");
    QuadtreeROS quadtree(nh,point_lower,point_upper,ds,false);

    ros::Time start_load = ros::Time::now();
    quadtree.load("test_quadtree_LNDARE");
    ros::Time done_load = ros::Time::now();
    std::cout << "Time to load: " << ros::Duration(done_load-start_load).toSec() << std::endl;

    jcv_diagram diagram;
    memset(&diagram, 0, sizeof(jcv_diagram));
    jcv_diagram_generate(points_vec.size(), points_vec.data(), 0, 0, &diagram);


    const jcv_edge* edges = jcv_diagram_get_edges(&diagram);
    int i = 0;

    OGRPoint check_point_a;
    OGRPoint check_point_b;
    OGRGeometry* geom;
    int total_edges = 0;
    int fast_check_edges = 0;
    int branches_pruned = 0;

    std::unordered_map<std::pair<double,double>,bool,hash_pair> contained_in_geometry_lookup;

    MapService map_service;
    GeographicLib::Geodesic geod(GeographicLib::Geodesic::WGS84());

    std::string voronoi_path = ros::package::getPath("voroni")+"/data/test_map/voronoi.sqlite";
    GDALDriver* sqlite_driver = GetGDALDriverManager()->GetDriverByName("SQLite");
    GDALDataset* voronoi_ds = sqlite_driver->Create(voronoi_path.c_str(),0,0,0,GDT_Unknown,NULL);
    

    if(voronoi_ds==NULL){
        ROS_ERROR_STREAM("Creation of sqlite dataset failed");
        return -1;
    }
 
    OGRLayer* voronoi_layer = voronoi_ds->CreateLayer("voronoi",nullptr,wkbMultiLineString);
    OGRFeature* voronoi_feature = OGRFeature::CreateFeature(voronoi_layer->GetLayerDefn());
    voronoi_feature->SetFID(0);
    OGRMultiLineString voronoi_geom;

    
    while(edges){
        bool intersects = false;
        total_edges++;
        if(total_edges%1000==0){
            std::cout << "Total edges processed: " << total_edges << std::endl;
        }

        if(quadtree.getLeafRegionContaining(edges->pos[0].x,edges->pos[0].y)==nullptr || quadtree.getLeafRegionContaining(edges->pos[1].x,edges->pos[1].y)==nullptr){
            fast_check_edges++;
            edges = jcv_diagram_get_next_edge(edges);
            continue;
        }
        
        //Try to prune branches that are approximately normal to land
        double edge_length;
        geod.Inverse(edges->pos[0].y,edges->pos[0].x,edges->pos[1].y,edges->pos[1].x,edge_length);
        edge_length=abs(edge_length);

        double d0=map_service.distance(edges->pos[0].x,edges->pos[0].y,LayerID::COLLISION);
        double d1=map_service.distance(edges->pos[1].x,edges->pos[1].y,LayerID::COLLISION);
        double diff = abs(abs(d1)-abs(d0))*1e5;
        //std::cout << abs(edge_length/diff) << std::endl;
        
        if(abs(edge_length/diff)<=1.25){
            branches_pruned++;
            edges = jcv_diagram_get_next_edge(edges);
            continue;
        }
        

        //std::cout << "Edge length: " << edge_length << " Diff: " << diff*1e5 << " Difference: " << abs(edge_length-diff) << std::endl;

        
        


        
        
        /*
        auto contained_in_geometry_a = contained_in_geometry_lookup.find(std::make_pair(edges->pos[0].x,edges->pos[0].y));
        auto contained_in_geometry_b = contained_in_geometry_lookup.find(std::make_pair(edges->pos[1].x,edges->pos[1].y));

        if(contained_in_geometry_a!=contained_in_geometry_lookup.end() || contained_in_geometry_b!=contained_in_geometry_lookup.end()){
            //One of the points are registered to be in land geometry, can skip check and set intersects=true
            intersects=true;
        } 
        */
       
        if(false){
            std::cout << "Should never end up here" << std::endl;
        }else{
            check_point_a.setX(edges->pos[0].x);
            check_point_a.setY(edges->pos[0].y);
            check_point_b.setX(edges->pos[1].x);
            check_point_b.setY(edges->pos[1].y);

            comparison_layer->ResetReading();
            while((feat = comparison_layer->GetNextFeature()) != NULL){
                geom = feat->GetGeometryRef();
                bool check_point_a_intersects = geom->Contains(&check_point_a);
                bool check_point_b_intersects = geom->Contains(&check_point_b);
                
                if(check_point_a_intersects){
                    contained_in_geometry_lookup.insert(std::make_pair(std::make_pair(edges->pos[0].x,edges->pos[0].y),true));
                    intersects=true;
                }
                if(check_point_b_intersects){
                    contained_in_geometry_lookup.insert(std::make_pair(std::make_pair(edges->pos[1].x,edges->pos[1].y),true));
                    intersects=true;
                }
                if(intersects) break;
                OGRFeature::DestroyFeature(feat);
            }
        }
        
        
        
        
        if (intersects){
            edges = jcv_diagram_get_next_edge(edges);
            continue;
        }
        
        
        //If comes this far, add
        OGRLineString line;
        line.addPoint(&check_point_a);
        line.addPoint(&check_point_b);
        voronoi_geom.addGeometry(&line);

        i++;
        edge_file << edges->pos[0].x << "," << edges->pos[0].y << "," << edges->pos[1].x << "," << edges->pos[1].y << "\n";
        edges = jcv_diagram_get_next_edge(edges);
    }
    voronoi_feature->SetGeometry(&voronoi_geom);
    voronoi_layer->CreateFeature(voronoi_feature);

    std::cout << "Number of edges: " << i << std::endl;
    std::cout << "Number of fast_check edges: " << fast_check_edges << std::endl;
    std::cout << "Branches pruned: "<< branches_pruned << std::endl;
    std::cout << "Original edges: "<< total_edges << std::endl;
    edge_file.close();

    jcv_diagram_free( &diagram );
}