//
//  Coord.h
//  gcj02
//

#ifndef Coord_h
#define Coord_h

#ifdef __cplusplus
extern "C" {
#endif
/**
 * WGS84坐标转国测局02坐标
 *
 * @param wg_flag 传0
 * @param wg_lng WGS84 经度
 * @param wg_lat WGS84 维度
 * @param wg_heit WGS84 高程（目前要求<8000).不要求准确值
 * @param wg_week GPS周 可以传0
 * @param wg_time GPS秒 可以传0
 * @param china_lng 变形后经度
 * @param china_lat 变形后维度
 *
 * @return
 */
unsigned int wgtochina_lb(int wg_flag, double wg_lng, double wg_lat,
                          int wg_heit, int wg_week, unsigned int wg_time,
                          double *china_lng, double *china_lat);
#ifdef __cplusplus
};
#endif

#endif /* Coord_h */
