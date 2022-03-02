import matplotlib.pyplot as plt
import matplotlib.collections  as mc
import pandas as pd
import numpy as np
from tqdm import tqdm
from osgeo import ogr, osr, gdal
from descartes import PolygonPatch
from shapely.geometry import Polygon
from shapely.wkt import loads
import rospkg

try:
    # installed with "pip install SciencePLots" (https://github.com/garrettj403/SciencePlots.git)
    # gives quite nice plots
    plt_styles = ["science", "grid", "bright", "no-latex"]
    plt.style.use(plt_styles)
    print(f"pyplot using style set {plt_styles}")
except Exception as e:
    plt.rcParams.update(
        {
            # setgrid
            "axes.grid": True,
            "grid.linestyle": ":",
            "grid.color": "k",
            "grid.alpha": 0.5,
            "grid.linewidth": 0.1,
            # Legend
            "legend.frameon": True,
            "legend.framealpha": 1.0,
            "legend.fancybox": True,
            "legend.numpoints": 1,
            "legend.loc" : "upper right",
            'legend.fontsize': 15,
            # Font
            "font.size" : 15,
            #Subplots and figure
            "figure.figsize" : [8,7],
            "figure.subplot.wspace" : 0.37,
            "figure.subplot.hspace" : 0.76,
            "figure.subplot.top" : 0.9,
            "figure.subplot.right" : 0.95,
            "figure.subplot.left" : 0.1,
        }
    )

BLUE = '#6699cc'
GRAY = '#999999'
GREEN = '#4F7942'

def main():
    rospack = rospkg.RosPack()
    figure,ax = plt.subplots(1,1)
    #Plot background
    datasource_path = rospack.get_path('voroni')+"/data/test_map/check_db.sqlite"
    ds:gdal.Dataset = gdal.OpenEx(datasource_path)
    if ds==None:
        raise RuntimeError("Failed to load datasource",datasource_path)
    collision_layer:ogr.Layer = ds.GetLayerByName("collision_dissolved")

    #Plot background
    collision_layer.ResetReading()
    for feat in collision_layer:
        for geom in feat.GetGeometryRef():
            wkt = geom.ExportToWkt()
            poly:Polygon = Polygon(loads(wkt))
            patch1 = PolygonPatch(poly, fc=GREEN, ec=GREEN, alpha=0.5, zorder=2)
            ax.add_patch(patch1)

    #Plot quadtree
    quadtree_path = rospack.get_path('usv_mission_planner')+"/data/quadtrees/test_quadtree_LNDARE/test_quadtree_LNDARE.csv"
    quadtree_df = pd.read_csv(quadtree_path)
    lines = []
    for index,row in quadtree_df.iterrows():
        line = [(row["u_lon"],row["u_lat"]),(row["v_lon"],row["v_lat"])]
        lines.append(line)
    lc = mc.LineCollection(lines, linewidths=0.1)
    ax.add_collection(lc)

    #Voroni edges
    quadtree_path = rospack.get_path('voroni')+"/data/debug/edges.csv"
    quadtree_df = pd.read_csv(quadtree_path)
    lines = []
    for index,row in quadtree_df.iterrows():
        line = [(row["x_from"],row["y_from"]),(row["x_to"],row["y_to"])]
        lines.append(line)
    lc = mc.LineCollection(lines, linewidths=1)
    ax.add_collection(lc)

    #Points
    points_path = rospack.get_path('voroni')+"/data/debug/points.csv"
    points_df = pd.read_csv(points_path)
    ax.scatter(points_df["x"],points_df["y"])




    plt.autoscale(enable=True, axis="both", tight=None)
    plt.show()

main()