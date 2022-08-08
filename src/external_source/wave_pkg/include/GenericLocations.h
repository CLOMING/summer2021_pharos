/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ITIS"
 * 	found in "../asn1c/J2735_201603DA+ITSK4-0.4_fix.update"
 * 	`asn1c -no-gen-example -fcompound-names -funnamed-unions -pdu=MessageFrame`
 */

#ifndef	_GenericLocations_H_
#define	_GenericLocations_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum GenericLocations {
	GenericLocations_on_bridges	= 7937,
	GenericLocations_in_tunnels	= 7938,
	GenericLocations_entering_or_leaving_tunnels	= 7939,
	GenericLocations_on_ramps	= 7940,
	GenericLocations_in_road_construction_area	= 7941,
	GenericLocations_around_a_curve	= 7942,
	GenericLocations_on_curve	= 8026,
	GenericLocations_on_tracks	= 8009,
	GenericLocations_in_street	= 8025,
	GenericLocations_shoulder	= 8027,
	GenericLocations_on_minor_roads	= 7943,
	GenericLocations_in_the_opposing_lanes	= 7944,
	GenericLocations_adjacent_to_roadway	= 7945,
	GenericLocations_across_tracks	= 8024,
	GenericLocations_on_bend	= 7946,
	GenericLocations_intersection	= 8032,
	GenericLocations_entire_intersection	= 7947,
	GenericLocations_in_the_median	= 7948,
	GenericLocations_moved_to_side_of_road	= 7949,
	GenericLocations_moved_to_shoulder	= 7950,
	GenericLocations_on_the_roadway	= 7951,
	GenericLocations_dip	= 8010,
	GenericLocations_traffic_circle	= 8011,
	GenericLocations_crossover	= 8028,
	GenericLocations_cross_road	= 8029,
	GenericLocations_side_road	= 8030,
	GenericLocations_to	= 8014,
	GenericLocations_by	= 8015,
	GenericLocations_through	= 8016,
	GenericLocations_area_of	= 8017,
	GenericLocations_under	= 8018,
	GenericLocations_over	= 8019,
	GenericLocations_from	= 8020,
	GenericLocations_approaching	= 8021,
	GenericLocations_entering_at	= 8022,
	GenericLocations_exiting_at	= 8023,
	GenericLocations_in_shaded_areas	= 7952,
	GenericLocations_in_low_lying_areas	= 7953,
	GenericLocations_in_the_downtown_area	= 7954,
	GenericLocations_in_the_inner_city_area	= 7955,
	GenericLocations_in_parts	= 7956,
	GenericLocations_in_some_places	= 7957,
	GenericLocations_in_the_ditch	= 7958,
	GenericLocations_in_the_valley	= 7959,
	GenericLocations_on_hill_top	= 7960,
	GenericLocations_near_the_foothills	= 7961,
	GenericLocations_at_high_altitudes	= 7962,
	GenericLocations_near_the_lake	= 7963,
	GenericLocations_near_the_shore	= 7964,
	GenericLocations_nearby_basin	= 8008,
	GenericLocations_over_the_crest_of_a_hill	= 7965,
	GenericLocations_other_than_on_the_roadway	= 7966,
	GenericLocations_near_the_beach	= 7967,
	GenericLocations_near_beach_access_point	= 7968,
	GenericLocations_mountain_pass	= 8006,
	GenericLocations_lower_level	= 7969,
	GenericLocations_upper_level	= 7970,
	GenericLocations_airport	= 7971,
	GenericLocations_concourse	= 7972,
	GenericLocations_gate	= 7973,
	GenericLocations_baggage_claim	= 7974,
	GenericLocations_customs_point	= 7975,
	GenericLocations_reservation_center	= 8007,
	GenericLocations_station	= 7976,
	GenericLocations_platform	= 7977,
	GenericLocations_dock	= 7978,
	GenericLocations_depot	= 7979,
	GenericLocations_ev_charging_point	= 7980,
	GenericLocations_information_welcome_point	= 7981,
	GenericLocations_at_rest_area	= 7982,
	GenericLocations_at_service_area	= 7983,
	GenericLocations_at_weigh_station	= 7984,
	GenericLocations_roadside_park	= 8033,
	GenericLocations_picnic_areas	= 7985,
	GenericLocations_rest_area	= 7986,
	GenericLocations_service_stations	= 7987,
	GenericLocations_toilets	= 7988,
	GenericLocations_bus_stop	= 8031,
	GenericLocations_park_and_ride_lot	= 8012,
	GenericLocations_on_the_right	= 7989,
	GenericLocations_on_the_left	= 7990,
	GenericLocations_in_the_center	= 7991,
	GenericLocations_in_the_opposite_direction	= 7992,
	GenericLocations_cross_traffic	= 7993,
	GenericLocations_northbound_traffic	= 7994,
	GenericLocations_eastbound_traffic	= 7995,
	GenericLocations_southbound_traffic	= 7996,
	GenericLocations_westbound_traffic	= 7997,
	GenericLocations_north	= 7998,
	GenericLocations_south	= 7999,
	GenericLocations_east	= 8000,
	GenericLocations_west	= 8001,
	GenericLocations_northeast	= 8002,
	GenericLocations_northwest	= 8003,
	GenericLocations_southeast	= 8004,
	GenericLocations_southwest	= 8005
	/*
	 * Enumeration is extensible
	 */
} e_GenericLocations;

/* GenericLocations */
typedef long	 GenericLocations_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_GenericLocations_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_GenericLocations;
extern const asn_INTEGER_specifics_t asn_SPC_GenericLocations_specs_1;
asn_struct_free_f GenericLocations_free;
asn_struct_print_f GenericLocations_print;
asn_constr_check_f GenericLocations_constraint;
ber_type_decoder_f GenericLocations_decode_ber;
der_type_encoder_f GenericLocations_encode_der;
xer_type_decoder_f GenericLocations_decode_xer;
xer_type_encoder_f GenericLocations_encode_xer;
oer_type_decoder_f GenericLocations_decode_oer;
oer_type_encoder_f GenericLocations_encode_oer;
per_type_decoder_f GenericLocations_decode_uper;
per_type_encoder_f GenericLocations_encode_uper;

#ifdef __cplusplus
}
#endif

#endif	/* _GenericLocations_H_ */
#include <asn_internal.h>
