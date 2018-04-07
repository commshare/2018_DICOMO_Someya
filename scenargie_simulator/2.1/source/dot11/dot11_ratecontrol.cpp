// Copyright (c) 2007-2016 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#include "dot11_ratecontrol.h"
#include "dot11_mac.h"

namespace Dot11 {
const string AdaptiveRateController::modelName = "Dot11Mac";

const unsigned int AdaptiveRateControllerWithMinstrelHT::primaryThroughputEntryIndex = 0;
const unsigned int AdaptiveRateControllerWithMinstrelHT::secondaryThroughputEntryIndex = 1;

template <typename T>
static inline
uint8_t ConvertToUInt8(const T& aValue)
{
    assert(aValue >= 0);
    assert(aValue <= static_cast<T>(255));

    return static_cast<uint8_t>(aValue);
}//ConvertToUInt8//

template <typename T>
static inline
uint16_t ConvertToUInt16(const T& aValue)
{
    assert(aValue >= 0);
    assert(aValue <= static_cast<T>(65535));

    return static_cast<uint16_t>(aValue);
}//ConvertToUInt16//



void AdaptiveRateController::OutputTraceForTxDatarateUpdate(const MacAddressType& destMacAddress)
{
    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {

        TransmissionParametersType txParameters;

        (*this).GetDataRateInfoForDataFrameToStation(
            destMacAddress,
            0,  // Use AC 0.
            0,  // Use First Tx rate.
            txParameters);

        const DatarateBitsPerSecType datarateBitsPerSec =
            macDurationCalculationInterfacePtr->CalcDatarateBitsPerSecond(txParameters);

        if (simEngineInterfacePtr->BinaryOutputIsOn()) {

            MacTxDatarateUpdateTraceRecord traceData;

            traceData.txDatarateBps = datarateBitsPerSec;
            traceData.destNodeId = destMacAddress.ExtractNodeId();

            assert(sizeof(traceData) == MAC_TX_DATARATE_UPDATE_TRACE_RECORD_BYTES);

            simEngineInterfacePtr->OutputTraceInBinary(modelName, interfaceId, "TxRateUpdate", traceData);
        }
        else {

            ostringstream msgStream;

            msgStream << "TxRate= " << datarateBitsPerSec;

            msgStream << " Dest= " << destMacAddress.ExtractNodeId();

            simEngineInterfacePtr->OutputTrace(modelName, interfaceId, "TxRateUpdate", msgStream.str());

        }//if//

    }//if//

}//OutputTraceForTxDatarateUpdate//



AdaptiveRateControllerWithMinstrelHT::AdaptiveRateControllerWithMinstrelHT(
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeIdType& nodeId,
    const InterfaceIdType& initInterfaceId,
    const unsigned int initInterfaceIndex,
    const unsigned int initBaseChannelBandwidthMhz,
    const unsigned int initMaxChannelBandwidthMhz,
    const bool initHighThroughputModeIsOn,
    const shared_ptr<Dot11MacDurationCalculationInterface>& initMacDurationCalculationInterfacePtr,
    const RandomNumberGeneratorSeedType& initNodeSeed)
    :
    AdaptiveRateController(
        initSimulationEngineInterfacePtr,
        initInterfaceId,
        initMacDurationCalculationInterfacePtr),
    aRandomNumberGenerator(
        HashInputsToMakeSeed(initNodeSeed, initInterfaceIndex)),
    baseChannelBandwidthMhz(initBaseChannelBandwidthMhz),
    maxChannelBandwidthMhz(initMaxChannelBandwidthMhz),
    highThroughputModeIsOn(initHighThroughputModeIsOn),
    typicalTransmissionUnitLengthBytes(1200),
    maxRetryCount(7),
    minRetryCount(2),
    samplingRetryCount(1),
    exponentiallyWeight(0.75),
    goodSuccessRate(0.95),
    badSuccessRate(0.20),
    worstSuccessRate(0.10),
    maxFrameSuccessRateToEstimateThroughput(0.9),
    samplingResultUpdateInterval(100*MILLI_SECOND),
    maxTransmissionDurationForAMultirateRetryStage(6*MILLI_SECOND),
    samplingTryingThreshold(20),
    maxLowRateSamplingCount(2),
    enoughNumberOfTransmittedMpdusToSample(30),
    samplingTransmissionInterval(16),
    initialSamplingTransmissionCount(4),
    ackDatarateController(theParameterDatabaseReader, nodeId, interfaceId),
    maxRetrySequenceIndex(MaxNumberAccessCategories - 1)
{
    lowestModulationAndCoding = minModulationAndCodingScheme;

    forceUseOfHighThroughputFrames = false;


    if (highThroughputModeIsOn) {
        if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "force-use-of-high-throughput-frames"), nodeId, interfaceId)) {

            forceUseOfHighThroughputFrames =
                theParameterDatabaseReader.ReadBool(
                    (parameterNamePrefix + "force-use-of-high-throughput-frames"),
                    nodeId, interfaceId);
        }//if//
    }//if//

    //management
    if (theParameterDatabaseReader.ParameterExists(
            "dot11-modulation-and-coding-for-management-frames", nodeId, interfaceId)) {

        modAndCodingForManagementFrames =
            ConvertNameToModulationAndCodingScheme(
                theParameterDatabaseReader.ReadString(
                    "dot11-modulation-and-coding-for-management-frames", nodeId, interfaceId),
                "dot11-modulation-and-coding-for-management-frames");
    }
    else {
        modAndCodingForManagementFrames = lowestModulationAndCoding;
    }//if//


    //broadcast
    if (theParameterDatabaseReader.ParameterExists(
            "dot11-modulation-and-coding-for-broadcast", nodeId, interfaceId)) {

        modAndCodingForBroadcast =
            ConvertNameToModulationAndCodingScheme(
                theParameterDatabaseReader.ReadString(
                    "dot11-modulation-and-coding-for-broadcast", nodeId, interfaceId),
                "dot11-modulation-and-coding-for-broadcast");
    }
    else {
        modAndCodingForBroadcast = lowestModulationAndCoding;
    }//if//

    //unicast
    const string modAndCodingListString =
        theParameterDatabaseReader.ReadString(parameterNamePrefix + "modulation-and-coding-list", nodeId, interfaceId);

    vector<string> modAndCodingNameVector;
    bool isSuccess;
    ConvertAStringIntoAVectorOfStrings(modAndCodingListString, isSuccess, modAndCodingNameVector);

    if ((!isSuccess) || (modAndCodingNameVector.empty())) {
        cerr << "Error in \"dot11-modulation-and-coding-list\" parameter was invalid." << endl;
        exit(1);
    }//if//

    modulationAndCodingSchemes.resize(modAndCodingNameVector.size());

    for(unsigned int i = 0; (i < modAndCodingNameVector.size()); i++) {
        bool success;
        GetModulationAndCodingSchemeFromName(
            modAndCodingNameVector[i],
            success,
            modulationAndCodingSchemes[i]);

        if (!success) {
            cerr << "Error in \"dot11-modulation-and-coding-list\" value: "
                 << modAndCodingNameVector[i] << " was invalid." << endl;
            exit(1);
        }//if//

        if ((i != 0) && (modulationAndCodingSchemes[i-1] >= modulationAndCodingSchemes[i])) {
            cerr << "Error in \"dot11-modulation-and-coding-list\". Schemes must be in "
                 << "increasing speed order." << endl;
            exit(1);
        }//if//

    }//for//

    const unsigned int maxNumberOfBondedChannels =
        maxChannelBandwidthMhz/baseChannelBandwidthMhz;

    unsigned int maxNumMimoSpatialStreams = MaxNumMimoSpatialStreams;

    if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "minstrel-ht-max-number-of-spatial-streams"), nodeId, interfaceId)) {

        maxNumMimoSpatialStreams =
            theParameterDatabaseReader.ReadInt(
                (parameterNamePrefix + "minstrel-ht-max-number-of-spatial-streams"), nodeId, interfaceId);

        if (maxNumMimoSpatialStreams > MaxNumMimoSpatialStreams) {
            cerr << "Error in \"dot11-minstrel-ht-max-number-of-spatial-streams\". Max number of Spatial Stream is "<< MaxNumMimoSpatialStreams << "." << endl;
            exit(1);
        }//if//
    }//if//

    for(unsigned int numberOfBondedChannels = 1; numberOfBondedChannels <= maxNumberOfBondedChannels; numberOfBondedChannels*= 2) {

        for(unsigned int numberOfStreams = 1; numberOfStreams <= maxNumMimoSpatialStreams; numberOfStreams++) {

            for(unsigned int modulationAndCodingSchemeIndex = 0; modulationAndCodingSchemeIndex < modulationAndCodingSchemes.size(); modulationAndCodingSchemeIndex++) {

                const ModulationAndCodingSchemesType& modulationAndCodingScheme =
                    modulationAndCodingSchemes[modulationAndCodingSchemeIndex];


                TransmissionParametersType txParameters;

                txParameters.bandwidthNumberChannels = numberOfBondedChannels;
                txParameters.modulationAndCodingScheme = modulationAndCodingScheme;
                txParameters.numberSpatialStreams = numberOfStreams;

                if (numberOfBondedChannels == 1) {
                    txParameters.isHighThroughputFrame = highThroughputModeIsOn;
                }
                else {
                    assert(numberOfBondedChannels >= 2);
                    txParameters.isHighThroughputFrame = true;
                }//if//

                modulationAndCodingSchemeEntries.push_back(
                    ModulationAndCodingSchemeEntry(
                        modulationAndCodingScheme,
                        ConvertToUInt8(numberOfStreams),
                        ConvertToUInt8(numberOfBondedChannels),
                        macDurationCalculationInterfacePtr->CalcDatarateBitsPerSecond(txParameters)));

                const ModulationAndCodingSchemeEntry& mcsEntry = modulationAndCodingSchemeEntries.back();

                (*this).RecalculateFrameRetryCount(
                    txParameters,
                    modulationAndCodingSchemeEntries.back());

            }//for//
        }//for//
    }//for//

    if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "minstrel-ht-typical-transmission-unit-length-bytes"), nodeId, interfaceId)) {
        typicalTransmissionUnitLengthBytes =
            theParameterDatabaseReader.ReadInt(
                (parameterNamePrefix + "minstrel-ht-typical-transmission-unit-length-bytes"), nodeId, interfaceId);
    }//if//

    if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "minstrel-ht-max-retry-count"), nodeId, interfaceId)) {
        maxRetryCount = ConvertToUInt8(
            theParameterDatabaseReader.ReadInt(
                (parameterNamePrefix + "minstrel-ht-max-retry-count"), nodeId, interfaceId));
    }//if//

    if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "minstrel-ht-min-retry-count"), nodeId, interfaceId)) {

        minRetryCount = ConvertToUInt8(
            theParameterDatabaseReader.ReadInt(
                (parameterNamePrefix + "minstrel-ht-min-retry-count"), nodeId, interfaceId));
    }//if//

    if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "minstrel-ht-sampling-retry-count"), nodeId, interfaceId)) {

        samplingRetryCount = ConvertToUInt8(
            theParameterDatabaseReader.ReadInt(
                (parameterNamePrefix + "minstrel-ht-sampling-retry-count"), nodeId, interfaceId));
    }//if//

    if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "minstrel-ht-moving-average-exponentially-weight"), nodeId, interfaceId)) {

        exponentiallyWeight =
            theParameterDatabaseReader.ReadDouble(
                (parameterNamePrefix + "minstrel-ht-moving-average-exponentially-weight"), nodeId, interfaceId);
    }//if//

    if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "minstrel-ht-good-success-rate"), nodeId, interfaceId)) {

        goodSuccessRate =
            theParameterDatabaseReader.ReadDouble(
                (parameterNamePrefix + "minstrel-ht-good-success-rate"), nodeId, interfaceId);
    }//if//

    if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "minstrel-ht-bad-success-rate"), nodeId, interfaceId)) {

        badSuccessRate =
            theParameterDatabaseReader.ReadDouble(
                (parameterNamePrefix + "minstrel-ht-bad-success-rate"), nodeId, interfaceId);
    }//if//

    if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "minstrel-ht-worst-success-rate"), nodeId, interfaceId)) {

        worstSuccessRate =
            theParameterDatabaseReader.ReadDouble(
                (parameterNamePrefix + "minstrel-ht-worst-success-rate"), nodeId, interfaceId);
    }//if//

    if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "minstrel-ht-max-success-rate-to-estimate-throughput"), nodeId, interfaceId)) {

        maxFrameSuccessRateToEstimateThroughput =
            theParameterDatabaseReader.ReadDouble(
                (parameterNamePrefix + "minstrel-ht-max-success-rate-to-estimate-throughput"), nodeId, interfaceId);
    }//if//

    if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "minstrel-ht-sampling-result-update-interval"), nodeId, interfaceId)) {

        samplingResultUpdateInterval =
            theParameterDatabaseReader.ReadTime(
                (parameterNamePrefix + "minstrel-ht-sampling-result-update-interval"), nodeId, interfaceId);
    }//if//

    if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "minstrel-ht-max-transmission-duration-for-a-multirate-retry-stage"), nodeId, interfaceId)) {

        maxTransmissionDurationForAMultirateRetryStage =
            theParameterDatabaseReader.ReadTime(
                (parameterNamePrefix + "minstrel-ht-max-transmission-duration-for-a-multirate-retry-stage"), nodeId, interfaceId);
    }//if//

    if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "minstrel-ht-low-rate-sampling-trying-threshold"), nodeId, interfaceId)) {

        samplingTryingThreshold = ConvertToUInt16(
            theParameterDatabaseReader.ReadInt(
                (parameterNamePrefix + "minstrel-ht-low-rate-sampling-trying-threshold"), nodeId, interfaceId));
    }//if//

    if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "minstrel-ht-max-low-rate-sampling-count"), nodeId, interfaceId)) {

        maxLowRateSamplingCount = ConvertToUInt16(
            theParameterDatabaseReader.ReadInt(
                (parameterNamePrefix + "minstrel-ht-max-low-rate-sampling-count"), nodeId, interfaceId));
    }//if//

    if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "minstrel-ht-enough-number-of-transmitted-mpdus-to-sample"), nodeId, interfaceId)) {

        enoughNumberOfTransmittedMpdusToSample = ConvertToUInt16(
            theParameterDatabaseReader.ReadInt(
                (parameterNamePrefix + "minstrel-ht-enough-number-of-transmitted-mpdus-to-sample"), nodeId, interfaceId));
    }//if//

    if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "minstrel-ht-sampling-transmission-interval-count"), nodeId, interfaceId)) {

        samplingTransmissionInterval = ConvertToUInt16(
            theParameterDatabaseReader.ReadInt(
                (parameterNamePrefix + "minstrel-ht-sampling-transmission-interval-count"), nodeId, interfaceId));
    }//if//

    if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "minstrel-ht-initial-sampling-transmission-count"), nodeId, interfaceId)) {

        initialSamplingTransmissionCount = ConvertToUInt16(
            theParameterDatabaseReader.ReadInt(
                (parameterNamePrefix + "minstrel-ht-initial-sampling-transmission-count"), nodeId, interfaceId));
    }//if//

}//AdaptiveRateControllerWithMinstrelHT//



void AdaptiveRateControllerWithMinstrelHT::RecalculateFrameRetryCount(
    const TransmissionParametersType& dataFrameTxParameters,
    ModulationAndCodingSchemeEntry& mcsEntry)
{
    TransmissionParametersType managementFrameTxParameters;

    managementFrameTxParameters.bandwidthNumberChannels = 1;
    managementFrameTxParameters.modulationAndCodingScheme = lowestModulationAndCoding;
    managementFrameTxParameters.numberSpatialStreams =1;
    managementFrameTxParameters.isHighThroughputFrame = highThroughputModeIsOn;

    const TimeType dataFrameTransmissionDuration =
        macDurationCalculationInterfacePtr->CalcMaxTxDurationForUnicastDataWithoutRts(
            typicalTransmissionUnitLengthBytes,
            dataFrameTxParameters,
            managementFrameTxParameters);

    const TimeType dataFrameTransmissionDurationWithRts =
        macDurationCalculationInterfacePtr->CalcMaxTxDurationForUnicastDataWithRts(
            typicalTransmissionUnitLengthBytes,
            dataFrameTxParameters,
            managementFrameTxParameters);

    assert(dataFrameTransmissionDuration < dataFrameTransmissionDurationWithRts);

    TimeType totalDataFrameTransmissionDuration = ZERO_TIME;
    TimeType totalDataFrameTransmissionDurationWithRts = ZERO_TIME;

    mcsEntry.frameRetryCount = 0;
    mcsEntry.frameRetryCountWithRts = 0;

    while (mcsEntry.frameRetryCount < maxRetryCount) {

        const TimeType averageBackoffDuration =
            macDurationCalculationInterfacePtr->CalcContentionWindowMinBackoffDuration(mcsEntry.frameRetryCount) / 2;

        totalDataFrameTransmissionDuration +=
            (averageBackoffDuration + dataFrameTransmissionDuration);

        totalDataFrameTransmissionDurationWithRts +=
            (averageBackoffDuration + dataFrameTransmissionDurationWithRts);

        if ((mcsEntry.frameRetryCount >= minRetryCount) &&
            (totalDataFrameTransmissionDuration >= maxTransmissionDurationForAMultirateRetryStage)) {
            break;
        }//if//

        mcsEntry.frameRetryCount++;

        if (totalDataFrameTransmissionDurationWithRts < maxTransmissionDurationForAMultirateRetryStage) {
            mcsEntry.frameRetryCountWithRts++;
        }//if//

    }//while//

}//RecalculateFrameRetryCount//


}//namespace
