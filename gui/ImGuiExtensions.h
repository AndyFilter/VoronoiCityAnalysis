#pragma once

#include "../External/ImGui/imgui.h"
#include "../structures.h"

#define LINE_BASE_COLOR ImGui::GetColorU32(ImGuiCol_Button)
#define LINE_SPECIAL_COLOR ImGui::GetColorU32(ImGuiCol_PlotHistogram)
#define LINE_INACTIVE_COLOR ImGui::GetColorU32(ImGuiCol_FrameBgHovered)
#define LINE_BASE_THICKNESS 2

#define POINT_BASE_COLOR LINE_BASE_COLOR
#define POINT_SPECIAL_COLOR LINE_SPECIAL_COLOR
#define POINT_BASE_RADIUS 8
#define POINT_CLOUD_BASE_RADIUS 2
//#define POINT_CLOUD_HULL_RADIUS 4

#define TREE_NODE_RADIUS 22
#define TREE_NODE_COLOR (0xFF593018)
#define TREE_NODE_LINE_COLOR (0xFFE09D4D) // E09D4D //0xFFB4B3BF
#define TREE_NODE_SPECIAL_COLOR (0xFF4623AB)
#define TREE_NODE_HEIGHT_2D 75
#define TREE_NODE_FRAME_BG 0xF0090909 // #55DDE0 //0xB0090909

#define SHAPE_FILL_COLOR (LINE_BASE_COLOR - 0xCC000000)

#define DELETE_BUTTON_HUE 0.988f //0xFFD33F49

#define CANVAS_SIZE 10.f

namespace ImGui {
    bool DrawPoint(Vec2 pos, const char* label, ImDrawList* dl, float radius = POINT_BASE_RADIUS, ImU32 col = POINT_BASE_COLOR);
    void DrawCanvas(ImDrawList* dl, Vec2 pos, Vec2 size = {0,0});
    void DrawArrow(ImDrawList* dl, Vec2 start, Vec2 end, ImU32 col = LINE_BASE_COLOR, float thickness = 2);
    void DrawDistanceLine(Vec2 p1, Vec2 p2, ImDrawList* dl, float distance_if_known = FLT_MAX, ImU32 col = LINE_BASE_COLOR,
                          float thickness = 1);
    void DrawConvexHull(PointCloud cp, ImDrawList* dl, bool draw_outline = true, float point_size = POINT_CLOUD_BASE_RADIUS,
                        Vec2 size = {-1, -1}, Vec2 offset = {0, 0}, ImU32 hull_points_col = POINT_SPECIAL_COLOR,
                                Vec2 pos_offset = {0,0});
    int DrawTree1D(RangeTree1D* tree, Vec2 pos);
    int DrawTree2D(RangeTree2D* tree, Vec2 pos);

    // User Configured Parameters
    bool DirectionalLineParams(DirectionalLineFunc& func);
    bool GeneralLineParams(GeneralLineFunc& func, int idx = 0);
    bool PointParams(Vec2 &P1, int idx = 0, float width = -1);
    bool PolygonParameters(Polygon& poly, int idx = 0);

    // Helpers
    inline Vec2 GetGeneralFuncOffset(GeneralLineFunc& func) {
        return (func.A == 0 ? Vec2(0, -func.C / func.B) : Vec2(-func.C / func.A, 0));
        //short mean_val = func.A * func.B == 0 ? 1 : 2;
        //return Vec2(func.A == 0 ? 0 : (func.C / func.A), func.B == 0 ? 0 : (func.C / func.B)) / mean_val;
    }
    inline Vec2 GetGeneralLineDir(GeneralLineFunc& func, Vec2 canvasSize = {1,1}) {
        return {func.B, func.A * canvasSize.y / canvasSize.x};
    }

    inline Vec2 Local2Canvas(Vec2 pos, Vec2 size, Vec2 offset) {
        return Vec2(1, -1) * (pos / (CANVAS_SIZE*2)) * size + offset + (size/2);
    }
    inline Vec2 Canvas2Local(Vec2 pos, Vec2 size, Vec2 offset) {
        return Vec2(1, -1) * (pos - offset - (size/2)) * (CANVAS_SIZE*2) / size;
    }

    inline ImVec2 operator-(const ImVec2& l, const ImVec2& r) { return{ l.x - r.x, l.y - r.y }; }
    inline ImVec2 operator+(const ImVec2& l, const ImVec2& r) { return{ l.x + r.x, l.y + r.y }; }
};