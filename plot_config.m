PATH_IMG = "img";

set(groot, "DefaultLineLineWidth", 2)
set(groot, "DefaultFigureRenderer", "opengl");
set(groot, "DefaultFigurePosition", [0 0 500 500]);
set(groot, "DefaultFigureColor", [1 1 1]);
set(groot, "DefaultAxesFontSize", 14);
set(groot, "DefaultTextInterpreter", "latex");
set(groot, "DefaultLegendInterpreter", "latex")

COLOR.GRAY = [200 200 200]/255;
COLOR.LIGHTGRAY = [220 220 220]/255;
COLOR.NAVY = [84 65 121]/255;
COLOR.BLUE = [46, 203, 255]/255;
% COLOR.CYAN = [0 207 255]/255; % OG
COLOR.TEAL = [89, 255, 244]/255;
COLOR.GREEN = [23 215 160]/255;
COLOR.ORANGE = [255, 171, 76]/255;
COLOR.RED = [255 95 126]/255;
COLOR.PURPLE = [176 0 185]/255;

COLOR.MAP = [COLOR.BLUE; COLOR.RED; COLOR.ORANGE; COLOR.NAVY; COLOR.GREEN; COLOR.PURPLE];