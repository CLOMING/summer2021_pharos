/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "../asn1c/J2735_201603DA+ITSK4-0.4_fix.update"
 * 	`asn1c -no-gen-example -fcompound-names -funnamed-unions -pdu=MessageFrame`
 */

#include "Snapshot.h"

asn_TYPE_member_t asn_MBR_Snapshot_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct Snapshot, thePosition),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_FullPositionVector,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"thePosition"
		},
	{ ATF_POINTER, 2, offsetof(struct Snapshot, safetyExt),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_VehicleSafetyExtensions,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"safetyExt"
		},
	{ ATF_POINTER, 1, offsetof(struct Snapshot, dataSet),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_VehicleStatus,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"dataSet"
		},
};
static const int asn_MAP_Snapshot_oms_1[] = { 1, 2 };
static const ber_tlv_tag_t asn_DEF_Snapshot_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_Snapshot_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* thePosition */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* safetyExt */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 } /* dataSet */
};
asn_SEQUENCE_specifics_t asn_SPC_Snapshot_specs_1 = {
	sizeof(struct Snapshot),
	offsetof(struct Snapshot, _asn_ctx),
	asn_MAP_Snapshot_tag2el_1,
	3,	/* Count of tags in the map */
	asn_MAP_Snapshot_oms_1,	/* Optional members */
	2, 0,	/* Root/Additions */
	3,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_Snapshot = {
	"Snapshot",
	"Snapshot",
	&asn_OP_SEQUENCE,
	asn_DEF_Snapshot_tags_1,
	sizeof(asn_DEF_Snapshot_tags_1)
		/sizeof(asn_DEF_Snapshot_tags_1[0]), /* 1 */
	asn_DEF_Snapshot_tags_1,	/* Same as above */
	sizeof(asn_DEF_Snapshot_tags_1)
		/sizeof(asn_DEF_Snapshot_tags_1[0]), /* 1 */
	{ 0, 0, SEQUENCE_constraint },
	asn_MBR_Snapshot_1,
	3,	/* Elements count */
	&asn_SPC_Snapshot_specs_1	/* Additional specs */
};

