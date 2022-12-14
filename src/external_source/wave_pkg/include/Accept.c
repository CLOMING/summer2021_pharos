/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ISO14827-2"
 * 	found in "../asn1c/J2735_201603DA+ITSK4-0.4_fix.update"
 * 	`asn1c -no-gen-example -fcompound-names -funnamed-unions -pdu=MessageFrame`
 */

#include "Accept.h"

static int
datexAccept_Packet_nbr_2_constraint(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	
	/* Constraint check succeeded */
	return 0;
}

/*
 * This type is implemented using NativeInteger,
 * so here we adjust the DEF accordingly.
 */
static int
registered_subscription_6_constraint(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	
	/* Constraint check succeeded */
	return 0;
}

/*
 * This type is implemented using NativeInteger,
 * so here we adjust the DEF accordingly.
 */
static int
memb_registered_subscription_constraint_3(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	
	/* Constraint check succeeded */
	return 0;
}

static int
memb_datexAccept_Packet_nbr_constraint_1(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	
	/* Constraint check succeeded */
	return 0;
}

static asn_oer_constraints_t asn_OER_type_datexAccept_Packet_nbr_constr_2 CC_NOTUSED = {
	{ 4, 1 }	/* (0..4294967295) */,
	-1};
static asn_per_constraints_t asn_PER_type_datexAccept_Packet_nbr_constr_2 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 32, -1,  0,  4294967295 }	/* (0..4294967295) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_oer_constraints_t asn_OER_type_registered_subscription_constr_6 CC_NOTUSED = {
	{ 4, 1 }	/* (0..4294967295) */,
	-1};
static asn_per_constraints_t asn_PER_type_registered_subscription_constr_6 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 32, -1,  0,  4294967295 }	/* (0..4294967295) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_oer_constraints_t asn_OER_memb_registered_subscription_constr_6 CC_NOTUSED = {
	{ 4, 1 }	/* (0..4294967295) */,
	-1};
static asn_per_constraints_t asn_PER_memb_registered_subscription_constr_6 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 32, -1,  0,  4294967295 }	/* (0..4294967295) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_oer_constraints_t asn_OER_type_datexAccept_Type_constr_3 CC_NOTUSED = {
	{ 0, 0 },
	-1};
static asn_per_constraints_t asn_PER_type_datexAccept_Type_constr_3 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 2,  2,  0,  3 }	/* (0..3) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_oer_constraints_t asn_OER_memb_datexAccept_Packet_nbr_constr_2 CC_NOTUSED = {
	{ 4, 1 }	/* (0..4294967295) */,
	-1};
static asn_per_constraints_t asn_PER_memb_datexAccept_Packet_nbr_constr_2 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 32, -1,  0,  4294967295 }	/* (0..4294967295) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static const asn_INTEGER_specifics_t asn_SPC_datexAccept_Packet_nbr_specs_2 = {
	0,	0,	0,	0,	0,
	0,	/* Native long size */
	1	/* Unsigned representation */
};
static const ber_tlv_tag_t asn_DEF_datexAccept_Packet_nbr_tags_2[] = {
	(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (2 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_datexAccept_Packet_nbr_2 = {
	"datexAccept-Packet-nbr",
	"datexAccept-Packet-nbr",
	&asn_OP_NativeInteger,
	asn_DEF_datexAccept_Packet_nbr_tags_2,
	sizeof(asn_DEF_datexAccept_Packet_nbr_tags_2)
		/sizeof(asn_DEF_datexAccept_Packet_nbr_tags_2[0]) - 1, /* 1 */
	asn_DEF_datexAccept_Packet_nbr_tags_2,	/* Same as above */
	sizeof(asn_DEF_datexAccept_Packet_nbr_tags_2)
		/sizeof(asn_DEF_datexAccept_Packet_nbr_tags_2[0]), /* 2 */
	{ &asn_OER_type_datexAccept_Packet_nbr_constr_2, &asn_PER_type_datexAccept_Packet_nbr_constr_2, datexAccept_Packet_nbr_2_constraint },
	0, 0,	/* No members */
	&asn_SPC_datexAccept_Packet_nbr_specs_2	/* Additional specs */
};

static const asn_INTEGER_specifics_t asn_SPC_registered_subscription_specs_6 = {
	0,	0,	0,	0,	0,
	0,	/* Native long size */
	1	/* Unsigned representation */
};
static const ber_tlv_tag_t asn_DEF_registered_subscription_tags_6[] = {
	(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (2 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_registered_subscription_6 = {
	"registered-subscription",
	"registered-subscription",
	&asn_OP_NativeInteger,
	asn_DEF_registered_subscription_tags_6,
	sizeof(asn_DEF_registered_subscription_tags_6)
		/sizeof(asn_DEF_registered_subscription_tags_6[0]) - 1, /* 1 */
	asn_DEF_registered_subscription_tags_6,	/* Same as above */
	sizeof(asn_DEF_registered_subscription_tags_6)
		/sizeof(asn_DEF_registered_subscription_tags_6[0]), /* 2 */
	{ &asn_OER_type_registered_subscription_constr_6, &asn_PER_type_registered_subscription_constr_6, registered_subscription_6_constraint },
	0, 0,	/* No members */
	&asn_SPC_registered_subscription_specs_6	/* Additional specs */
};

static asn_TYPE_member_t asn_MBR_datexAccept_Type_3[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct Accept__datexAccept_Type, logIn),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_OBJECT_IDENTIFIER,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"logIn"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct Accept__datexAccept_Type, single_subscription),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_NULL,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"single-subscription"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct Accept__datexAccept_Type, registered_subscription),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_registered_subscription_6,
		0,
		{ &asn_OER_memb_registered_subscription_constr_6, &asn_PER_memb_registered_subscription_constr_6,  memb_registered_subscription_constraint_3 },
		0, 0, /* No default value */
		"registered-subscription"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct Accept__datexAccept_Type, publication),
		(ASN_TAG_CLASS_CONTEXT | (3 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_NULL,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"publication"
		},
};
static const asn_TYPE_tag2member_t asn_MAP_datexAccept_Type_tag2el_3[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* logIn */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* single-subscription */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 }, /* registered-subscription */
    { (ASN_TAG_CLASS_CONTEXT | (3 << 2)), 3, 0, 0 } /* publication */
};
static asn_CHOICE_specifics_t asn_SPC_datexAccept_Type_specs_3 = {
	sizeof(struct Accept__datexAccept_Type),
	offsetof(struct Accept__datexAccept_Type, _asn_ctx),
	offsetof(struct Accept__datexAccept_Type, present),
	sizeof(((struct Accept__datexAccept_Type *)0)->present),
	asn_MAP_datexAccept_Type_tag2el_3,
	4,	/* Count of tags in the map */
	0, 0,
	-1	/* Extensions start */
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_datexAccept_Type_3 = {
	"datexAccept-Type",
	"datexAccept-Type",
	&asn_OP_CHOICE,
	0,	/* No effective tags (pointer) */
	0,	/* No effective tags (count) */
	0,	/* No tags (pointer) */
	0,	/* No tags (count) */
	{ &asn_OER_type_datexAccept_Type_constr_3, &asn_PER_type_datexAccept_Type_constr_3, CHOICE_constraint },
	asn_MBR_datexAccept_Type_3,
	4,	/* Elements count */
	&asn_SPC_datexAccept_Type_specs_3	/* Additional specs */
};

asn_TYPE_member_t asn_MBR_Accept_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct Accept, datexAccept_Packet_nbr),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_datexAccept_Packet_nbr_2,
		0,
		{ &asn_OER_memb_datexAccept_Packet_nbr_constr_2, &asn_PER_memb_datexAccept_Packet_nbr_constr_2,  memb_datexAccept_Packet_nbr_constraint_1 },
		0, 0, /* No default value */
		"datexAccept-Packet-nbr"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct Accept, datexAccept_Type),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		+1,	/* EXPLICIT tag at current level */
		&asn_DEF_datexAccept_Type_3,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"datexAccept-Type"
		},
};
static const ber_tlv_tag_t asn_DEF_Accept_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_Accept_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* datexAccept-Packet-nbr */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* datexAccept-Type */
};
asn_SEQUENCE_specifics_t asn_SPC_Accept_specs_1 = {
	sizeof(struct Accept),
	offsetof(struct Accept, _asn_ctx),
	asn_MAP_Accept_tag2el_1,
	2,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_Accept = {
	"Accept",
	"Accept",
	&asn_OP_SEQUENCE,
	asn_DEF_Accept_tags_1,
	sizeof(asn_DEF_Accept_tags_1)
		/sizeof(asn_DEF_Accept_tags_1[0]), /* 1 */
	asn_DEF_Accept_tags_1,	/* Same as above */
	sizeof(asn_DEF_Accept_tags_1)
		/sizeof(asn_DEF_Accept_tags_1[0]), /* 1 */
	{ 0, 0, SEQUENCE_constraint },
	asn_MBR_Accept_1,
	2,	/* Elements count */
	&asn_SPC_Accept_specs_1	/* Additional specs */
};

