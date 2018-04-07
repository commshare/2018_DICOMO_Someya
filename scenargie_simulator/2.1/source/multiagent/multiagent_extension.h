// Copyright (c) 2007-2016 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef MULTIAGENT_EXTENSION_H
#define MULTIAGENT_EXTENSION_H

#include "multiagent_agentsim.h"

namespace MultiAgent {

enum {
    //AGENT_TASK_PURPOSE_OTHER = 0,
    AGENT_TASK_PURPOSE_COMMUTE_TO_WORK = 1,
    AGENT_TASK_PURPOSE_COMMUTE_TO_SCHOOL,
    AGENT_TASK_PURPOSE_SHOPPING,
    AGENT_TASK_PURPOSE_PERSONAL_BUSINESS,
    AGENT_TASK_PURPOSE_WORK_RELATED_BUSINESS,
    AGENT_TASK_PURPOSE_SOCIAL_RECREATION,
    AGENT_TASK_PURPOSE_EVENT,
    AGENT_TASK_PURPOSE_SERVE_PASSENGER,
    AGENT_TASK_PURPOSE_EDUCATION,
    AGENT_TASK_PURPOSE_CHILD_CARE,
};

enum {
    //AGENT_USER_TYPE_NONE = 0,

    AGENT_USER_TYPE_WORKER = 1,
    AGENT_USER_TYPE_STUDENT,
    AGENT_USER_TYPE_CHILD,
    AGENT_USER_TYPE_HOUSE_PERSON,
    AGENT_USER_TYPE_VISITOR,
    AGENT_USER_TYPE_PENSIONER,
    AGENT_USER_TYPE_DISABLED_PERSON,
};

enum {
    //AGENT_MOBILTY_CLASS_NORMAL = 0,

    AGENT_MOBILTY_CLASS_SLOW = 1,
    AGENT_MOBILTY_CLASS_VERY_SLOW,
    AGENT_MOBILTY_CLASS_WEELCHAIR,

    NUMBER_AGENT_MOBILTY_CLASSES,
};

enum {
    //AGENT_TICKET_FULL_FARE = 0,
    
    AGENT_TICKET_CHILD_FARE = 1,
    AGENT_TICKET_FREE_PASS,
    AGENT_TICKET_CONCESSION_PENSIONER,
    AGENT_TICKET_CONCESSION_STUDENT,

    NUMBER_AGENT_TICKET_TYPES,
};

static inline
AgentTaskPurposeType GetAgentPurposeType(const string& purposeType)
{
    if (purposeType == "other") {
        return AGENT_TASK_PURPOSE_OTHER;
    } else if (purposeType == "commutetowork") {
        return AGENT_TASK_PURPOSE_COMMUTE_TO_WORK;
    } else if (purposeType == "commutetoschool") {
        return AGENT_TASK_PURPOSE_COMMUTE_TO_SCHOOL;
    } else if (purposeType == "shopping") {
        return AGENT_TASK_PURPOSE_SHOPPING;
    } else if (purposeType == "personalbusiness") {
        return AGENT_TASK_PURPOSE_PERSONAL_BUSINESS;
    } else if (purposeType == "workrelatedbusiness") {
        return AGENT_TASK_PURPOSE_WORK_RELATED_BUSINESS;
    } else if (purposeType == "socialrecreation") {
        return AGENT_TASK_PURPOSE_SOCIAL_RECREATION;
    } else if (purposeType == "event") {
        return AGENT_TASK_PURPOSE_EVENT;
    } else if (purposeType == "servepassenger") {
        return AGENT_TASK_PURPOSE_SERVE_PASSENGER;
    } else if (purposeType == "education") {
        return AGENT_TASK_PURPOSE_EDUCATION;
    } else if (purposeType == "childcare") {
        return AGENT_TASK_PURPOSE_CHILD_CARE;
    } else {
        cerr << "Error: invalid purpose " << purposeType << endl;
        exit(1);
    }

    return AGENT_TASK_PURPOSE_OTHER;
}

static inline
string GetAgentPurposeName(const AgentTaskPurposeType& purposeType)
{
    switch (purposeType) {
    case AGENT_TASK_PURPOSE_OTHER: return "other";
    case AGENT_TASK_PURPOSE_COMMUTE_TO_WORK: return "commutetowork";
    case AGENT_TASK_PURPOSE_COMMUTE_TO_SCHOOL: return "commutetoschool";
    case AGENT_TASK_PURPOSE_SHOPPING: return "shopping";
    case AGENT_TASK_PURPOSE_PERSONAL_BUSINESS: return "personalbusiness";
    case AGENT_TASK_PURPOSE_WORK_RELATED_BUSINESS: return "workrelatedbusiness";
    case AGENT_TASK_PURPOSE_SOCIAL_RECREATION: return "socialrecreation";
    case AGENT_TASK_PURPOSE_EVENT: return "event";
    case AGENT_TASK_PURPOSE_SERVE_PASSENGER: return "servepassenger";
    case AGENT_TASK_PURPOSE_EDUCATION: return "education";
    case AGENT_TASK_PURPOSE_CHILD_CARE: return "childcare";
    }

    return "other";
}

static inline
AgentUserType GetAgentUserType(const string& userType)
{
    if (userType == "worker") {
        return AGENT_USER_TYPE_WORKER;
    } else if (userType == "student") {
        return AGENT_USER_TYPE_STUDENT;
    } else if (userType == "child") {
        return AGENT_USER_TYPE_CHILD;
    } else if (userType == "houseperson") {
        return AGENT_USER_TYPE_HOUSE_PERSON;
    } else if (userType == "visitor") {
        return AGENT_USER_TYPE_VISITOR;
    } else if (userType == "pensioner") {
        return AGENT_USER_TYPE_PENSIONER;
    } else if (userType == "disabledperson") {
        return AGENT_USER_TYPE_DISABLED_PERSON;
    } else if (userType == "none") {
        return AGENT_USER_TYPE_NONE;
    } else {
        cerr << "Error: unkonwn user type " << userType << endl;
        exit(1);
    }

    return AGENT_USER_TYPE_NONE;
}

static inline
string GetAgentUserName(const AgentUserType& userType)
{
    switch (userType) {
    case AGENT_USER_TYPE_WORKER: return "worker";
    case AGENT_USER_TYPE_STUDENT: return "student";
    case AGENT_USER_TYPE_CHILD: return "child";
    case AGENT_USER_TYPE_HOUSE_PERSON: return "houseperson";
    case AGENT_USER_TYPE_VISITOR: return "visitor";
    case AGENT_USER_TYPE_PENSIONER: return "pensioner";
    case AGENT_USER_TYPE_DISABLED_PERSON: return "disabledperson";
    }

    return "worker";
}

static inline
bool IsUserTypeADisabledPerson(const AgentUserType& userType)
{
    return (userType == AGENT_USER_TYPE_DISABLED_PERSON);
}

static inline
AgentMobilityClassType GetAgentMobilityClass(const string& mobilityType)
{
    if (mobilityType == "normal") {
        return AGENT_MOBILTY_CLASS_NORMAL;
    } else if (mobilityType == "slow") {
        return AGENT_MOBILTY_CLASS_SLOW;
    } else if (mobilityType == "veryslow") {
        return AGENT_MOBILTY_CLASS_VERY_SLOW;
    } else if (mobilityType == "wheelchair") {
        return AGENT_MOBILTY_CLASS_WEELCHAIR;
    } else {
        cerr << "Error: unkonwn mobility class " << mobilityType << endl;
        exit(1);
    }

    return AGENT_MOBILTY_CLASS_NORMAL;
}

static inline
string GetAgentMobilityClassName(const AgentMobilityClassType& mobilityClassType)
{
    switch (mobilityClassType) {
    case AGENT_MOBILTY_CLASS_NORMAL: return "normal";
    case AGENT_MOBILTY_CLASS_SLOW: return "slow";
    case AGENT_MOBILTY_CLASS_VERY_SLOW: return "veryslow";
    case AGENT_MOBILTY_CLASS_WEELCHAIR: return "wheelchair";
    }

    return "normal";
}

static inline size_t GetNumberOfMobilityClasses() { return NUMBER_AGENT_MOBILTY_CLASSES; }

static inline
double GetTypicalWalkSpeed(const AgentMobilityClassType& mobilityClass)
{
    switch (mobilityClass) {
    case AGENT_MOBILTY_CLASS_NORMAL: return 3;
    case AGENT_MOBILTY_CLASS_SLOW: return 1;
    case AGENT_MOBILTY_CLASS_VERY_SLOW: return 0.5;
    case AGENT_MOBILTY_CLASS_WEELCHAIR: return 0.5;
    default: return 3;
    }

    return 3;
}

static inline
AgentTicketType GetAgentTicket(const string& ticketType)
{
    if (ticketType == "full") {
       return AGENT_TICKET_FULL_FARE;
    } else if (ticketType == "child") {
       return AGENT_TICKET_CHILD_FARE;
    } else if (ticketType == "freepass") {
       return AGENT_TICKET_FREE_PASS;
    } else if (ticketType == "concessionpensioner") {
       return AGENT_TICKET_CONCESSION_PENSIONER;
    } else if (ticketType == "concessionstudent") {
       return AGENT_TICKET_CONCESSION_STUDENT;
    } else {
        cerr << "Error: unkonwn ticket type " << ticketType << endl;
        exit(1);
    }

    return AGENT_TICKET_FULL_FARE;
}

static inline
string GetAgentTicketName(const AgentTicketType& ticketType)
{
    switch (ticketType) {
    case AGENT_TICKET_FULL_FARE: return "full";
    case AGENT_TICKET_CHILD_FARE: return "child";
    case AGENT_TICKET_FREE_PASS: return "freepass";
    case AGENT_TICKET_CONCESSION_PENSIONER: return "concessionpensioner";
    case AGENT_TICKET_CONCESSION_STUDENT: return "concessionstudent";
    }

    return "full";
}

static inline size_t GetNumberOfTickets() { return NUMBER_AGENT_TICKET_TYPES; }


} //namespace

#endif
