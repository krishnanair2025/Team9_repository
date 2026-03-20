from roboflow import Roboflow
rf = Roboflow(api_key="P4RTTDYhXSVz0TvM9SGn")
project = rf.workspace("leo-rover-zkfek").project("my-first-project-vtcgk")
version = project.version(2)
dataset = version.download("yolov7")
                
