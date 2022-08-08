/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "../asn1c/J2735_201603DA+ITSK4-0.4_fix.update"
 * 	`asn1c -no-gen-example -fcompound-names -funnamed-unions -pdu=MessageFrame`
 */

#include "SignalRequestPackage.h"

static int
memb_regional_constraint_1(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	size_t size;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	/* Determine the number of elements */
	size = _A_CSEQUENCE_FROM_VOID(sptr)->count;
	
	if((size >= 1 && size <= 4)) {
		/* Perform validation of the inner elements */
		return td->encoding_constraints.general_constraints(td, sptr, ctfailcb, app_key);
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static asn_oer_constraints_t asn_OER_type_regional_constr_6 CC_NOTUSED = {
	{ 0, 0 },
	-1	/* (SIZE(1..4)) */};
static asn_per_constraints_t asn_PER_type_regional_constr_6 CC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_CONSTRAINED,	 2,  2,  1,  4 }	/* (SIZE(1..4)) */,
	0, 0	/* No PER value map */
};
static asn_oer_constraints_t asn_OER_memb_regional_constr_6 CC_NOTUSED = {
	{ 0, 0 },
	-1	/* (SIZE(1..4)) */};
static asn_per_constraints_t asn_PER_memb_regional_constr_6 CC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_CONSTRAINED,	 2,  2,  1,  4 }	/* (SIZE(1..4)) */,
	0, 0	/* No PER value map */
};
static asn_TYPE_member_t asn_MBR_regional_6[] = {
	{ ATF_POINTER, 0, 0,
		(ASN_TAG_CLASS_UNIVERSAL | (16 << 2)),
		0,
		&asn_DEF_RegionalExtension_124P0,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		""
		},
};
static const ber_tlv_tag_t asn_DEF_regional_tags_6[] = {
	(ASN_TAG_CLASS_CONTEXT | (4 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static asn_SET_OF_specifics_t asn_SPC_regional_specs_6 = {
	sizeof(struct SignalRequestPackage__regional),
	offsetof(struct SignalRequestPackage__regional, _asn_ctx),
	0,	/* XER encoding is XMLDelimitedItemList */
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_regional_6 = {
	"regional",
	"regional",
	&asn_OP_SEQUENCE_OF,
	asn_DEF_regional_tags_6,
	sizeof(asn_DEF_regional_tags_6)
		/sizeof(asn_DEF_regional_tags_6[0]) - 1, /* 1 */
	asn_DEF_regional_tags_6,	/* Same as above */
	sizeof(asn_DEF_regional_tags_6)
		/sizeof(asn_DEF_regional_tags_6[0]), /* 2 */
	{ &asn_OER_type_regional_constr_6, &asn_PER_type_regional_constr_6, SEQUENCE_OF_constraint },
	asn_MBR_regional_6,
	1,	/* Single element */
	&asn_SPC_regional_specs_6	/* Additional specs */
};

asn_TYPE_member_t asn_MBR_SignalRequestPackage_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct SignalRequestPackage, request),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_SignalRequest,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"request"
		},
	{ ATF_POINTER, 4, offsetof(struct SignalRequestPackage, minute),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_MinuteOfTheYear,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"minute"
		},
	{ ATF_POINTER, 3, offsetof(struct SignalRequestPackage, second),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_DSecond,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"second"
		},
	{ ATF_POINTER, 2, offsetof(struct SignalRequestPackage, duration),
		(ASN_TAG_CLASS_CONTEXT | (3 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_DSecond,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"duration"
		},
	{ ATF_POINTER, 1, offsetof(struct SignalRequestPackage, regional),
		(ASN_TAG_CLASS_CONTEXT | (4 << 2)),
		0,
		&asn_DEF_regional_6,
		0,
		{ &asn_OER_memb_regional_constr_6, &asn_PER_memb_regional_constr_6,  memb_regional_constraint_1 },
		0, 0, /* No default value */
		"regional"
		},
};
static const int asn_MAP_SignalRequestPackage_oms_1[] = { 1, 2, 3, 4 };
static const ber_tlv_tag_t asn_DEF_SignalRequestPackage_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_SignalRequestPackage_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* request */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* minute */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 }, /* second */
    { (ASN_TAG_CLASS_CONTEXT | (3 << 2)), 3, 0, 0 }, /* duration */
    { (ASN_TAG_CLASS_CONTEXT | (4 << 2)), 4, 0, 0 } /* regional */
};
asn_SEQUENCE_specifics_t asn_SPC_SignalRequestPackage_specs_1 = {
	sizeof(struct SignalRequestPackage),
	offsetof(struct SignalRequestPackage, _asn_ctx),
	asn_MAP_SignalRequestPackage_tag2el_1,
	5,	/* Count of tags in the map */
	asn_MAP_SignalRequestPackage_oms_1,	/* Optional members */
	4, 0,	/* Root/Additions */
	5,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_SignalRequestPackage = {
	"SignalRequestPackage",
	"SignalRequestPackage",
	&asn_OP_SEQUENCE,
	asn_DEF_SignalRequestPackage_tags_1,
	sizeof(asn_DEF_SignalRequestPackage_tags_1)
		/sizeof(asn_DEF_SignalRequestPackage_tags_1[0]), /* 1 */
	asn_DEF_SignalRequestPackage_tags_1,	/* Same as above */
	sizeof(asn_DEF_SignalRequestPackage_tags_1)
		/sizeof(asn_DEF_SignalRequestPackage_tags_1[0]), /* 1 */
	{ 0, 0, SEQUENCE_constraint },
	asn_MBR_SignalRequestPackage_1,
	5,	/* Elements count */
	&asn_SPC_SignalRequestPackage_specs_1	/* Additional specs */
};

