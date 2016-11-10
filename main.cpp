#define END_TIME 5000

#define MEDIA_1 0
#define MEDIA_2 1
#define CHEMO_1 2
#define CHEMO_2 3
#define CELLSTAT 4
#define WASTE 5
#define CLEANING_WASTE 6
#define NaOH 7
#define ETHANOL 8
#define WATER 9
#define AIR 10

#include <QCoreApplication>

#include <easylogging++.h>

#include "fluidControl/executable/ExecutableMachineGraph.h"
#include "fluidControl/mapping/pathcalculator/PathManager.h"

#include "protocolGraph/ProtocolGraph.h"
#include "protocolGraph/OperationNode.h"
#include "protocolGraph/ConditionEdge.h"

#include "protocolGraph/operations/AssignationOperation.h"
#include "protocolGraph/operations/container/ApplyLight.h"
#include "protocolGraph/operations/container/ApplyTemperature.h"
#include "protocolGraph/operations/container/ContainerOperation.h"
#include "protocolGraph/operations/container/GetVolume.h"
#include "protocolGraph/operations/container/LoadContainerOperation.h"
#include "protocolGraph/operations/container/MeasureOD.h"
#include "protocolGraph/operations/container/Mix.h"
#include "protocolGraph/operations/container/SetContinousFlow.h"
#include "protocolGraph/operations/container/TimeStep.h"
#include "protocolGraph/operations/container/Transfer.h"
#include "protocolGraph/operations/DivergeNode.h"
#include "protocolGraph/operations/LoopNode.h"

#include "operables/mathematics/ConstantNumber.h"
#include "operables/mathematics/ArithmeticOperation.h"
#include "operables/mathematics/UnaryOperation.h"
#include "operables/mathematics/MathematicOperable.h"
#include "operables/mathematics/VariableEntry.h"

#include "operables/VariableTable.h"

#include "operables/comparison/ComparisonOperable.h"
#include "operables/comparison/Tautology.h"
#include "operables/comparison/SimpleComparison.h"
#include "operables/comparison/BooleanComparison.h"

#include "util/AutoEnumerate.h"
#include "util/logutils.h"

#include "fluidControl/ExecutionEngine.h"
#include "fluidControl/mapping/MappingEngine.h"
#include "fluidControl/machineGraph/MachineGraph.h"
#include "fluidControl/machineGraph/ContainerNode.h"
#include "fluidControl/machineGraph/ContainerNodeType.h"

#include "fluidControl/executable/containers/actuators/liquids/Control.h"
#include "fluidControl/executable/containers/actuators/liquids/Injector.h"
#include "fluidControl/executable/containers/actuators/liquids/Extractor.h"

#include "fluidControl/executable/containers/actuators/extras/Temperature.h"
#include "fluidControl/executable/containers/actuators/extras/ODSensor.h"
#include "fluidControl/executable/containers/actuators/extras/Light.h"
#include "fluidControl/executable/containers/actuators/extras/Mixer.h"

#include "fluidControl/executable/containers/actuators/communications/CommunicationsInterface.h"
#include "fluidControl/executable/containers/actuators/communications/CommandSender.h"
#include "fluidControl/executable/containers/actuators/communications/FileSender.h"
#include "fluidControl/executable/containers/actuators/communications/SerialSender.h"

#include "fluidControl/executable/containers/InletContainer.h"
#include "fluidControl/executable/containers/BidirectionalSwitch.h"
#include "fluidControl/executable/containers/ConvergentSwitch.h"
#include "fluidControl/executable/containers/ConvergentSwitchInlet.h"
#include "fluidControl/executable/containers/DivergentSwitch.h"
#include "fluidControl/executable/containers/DivergentSwitchSink.h"
#include "fluidControl/executable/containers/ExecutableContainerNode.h"
#include "fluidControl/executable/containers/FlowContainer.h"
#include "fluidControl/executable/containers/SinkContainer.h"

//plugins
#include "plugin/PluginFileLoader.h"
#include "plugin/PythonEnvironment.h"
#include "plugin/actuators/ODSensorPlugin.h"
#include "plugin/actuators/MixerPlugin.h"
#include "plugin/actuators/TemperaturePlugin.h"
#include "plugin/actuators/LightPlugin.h"
#include "plugin/actuators/ControlPlugin.h"
#include "plugin/actuators/ExtractorPlugin.h"
#include "plugin/actuators/InjectorPlugin.h"

INITIALIZE_NULL_EASYLOGGINGPP

using namespace std;

shared_ptr<OperationNode> createifOperation (int initTime,
                        int durationTime,
                        std::shared_ptr<OperationNode> yesBranch,
                        std::shared_ptr<OperationNode> noBranch,
                        AutoEnumerate & serial,
                        ProtocolGraph* protocol)
{
      shared_ptr<MathematicOperable> init =
            shared_ptr<MathematicOperable>(new ConstantNumber(initTime));
    shared_ptr<MathematicOperable> end =
            shared_ptr<MathematicOperable>(new ConstantNumber(initTime + durationTime));
    shared_ptr<MathematicOperable> vTime =
            shared_ptr<MathematicOperable>(new VariableEntry(TIME_VARIABLE));

    shared_ptr<ComparisonOperable> compareInit =
            shared_ptr<ComparisonOperable>(new SimpleComparison(false, vTime, comparison::greater, init));
    shared_ptr<ComparisonOperable> compareEnd =
            shared_ptr<ComparisonOperable>(new SimpleComparison(false, vTime, comparison::less_equal, end));
    shared_ptr<ComparisonOperable> andCompare =
            shared_ptr<ComparisonOperable>(new BooleanComparison(false, compareInit, logical::BooleanOperator::conjunction, compareEnd));
    shared_ptr<ComparisonOperable> notAndCompare =
            shared_ptr<ComparisonOperable>(new BooleanComparison(true, compareInit, logical::BooleanOperator::conjunction, compareEnd));

    shared_ptr<OperationNode> ifNode =
            shared_ptr<OperationNode>(new DivergeNode(serial.getNextValue(),andCompare));

    protocol->addOperation(ifNode);
    protocol->connectOperation(ifNode, yesBranch, andCompare);
    if (noBranch) {
        protocol->connectOperation(ifNode, noBranch, notAndCompare);
    }
    return ifNode;
}
shared_ptr<OperationNode> createifOperation (int initTime,
                        int durationTime,
                        std::shared_ptr<OperationNode> yesBranch,
                        std::shared_ptr<OperationNode> noBranch,
                        AutoEnumerate & serial,
                        ProtocolGraph* protocol,
                        shared_ptr<ComparisonOperable> & outCompare)
{
      shared_ptr<MathematicOperable> init =
            shared_ptr<MathematicOperable>(new ConstantNumber(initTime));
    shared_ptr<MathematicOperable> end =
            shared_ptr<MathematicOperable>(new ConstantNumber(initTime + durationTime));
    shared_ptr<MathematicOperable> vTime =
            shared_ptr<MathematicOperable>(new VariableEntry(TIME_VARIABLE));

    shared_ptr<ComparisonOperable> compareInit =
            shared_ptr<ComparisonOperable>(new SimpleComparison(false, vTime, comparison::greater, init));
    shared_ptr<ComparisonOperable> compareEnd =
            shared_ptr<ComparisonOperable>(new SimpleComparison(false, vTime, comparison::less_equal, end));
    shared_ptr<ComparisonOperable> andCompare =
            shared_ptr<ComparisonOperable>(new BooleanComparison(false, compareInit, logical::BooleanOperator::conjunction, compareEnd));
    shared_ptr<ComparisonOperable> notAndCompare =
            shared_ptr<ComparisonOperable>(new BooleanComparison(true, compareInit, logical::BooleanOperator::conjunction, compareEnd));

    shared_ptr<OperationNode> ifNode =
            shared_ptr<OperationNode>(new DivergeNode(serial.getNextValue(),andCompare));

    protocol->addOperation(ifNode);
    protocol->connectOperation(ifNode, yesBranch, andCompare);
    if (noBranch) {
        protocol->connectOperation(ifNode, noBranch, notAndCompare);
    } else {
        outCompare = notAndCompare;
    }

    return ifNode;
}

ProtocolGraph* makeTurbidostat() {
    AutoEnumerate serial;
    ProtocolGraph* protocol = new ProtocolGraph("turbidostat");

    std::shared_ptr<VariableEntry> epsilon(
        new VariableEntry("epsilon"));
    std::shared_ptr<VariableEntry> threshold(
        new VariableEntry("threshold"));
    std::shared_ptr<VariableEntry> rate(new VariableEntry("rate"));

    std::shared_ptr<MathematicOperable> mepsilon(
        new VariableEntry("epsilon"));
    std::shared_ptr<MathematicOperable> mthreshold(
        new VariableEntry("threshold"));
    std::shared_ptr<MathematicOperable> mrate(
        new VariableEntry("rate"));

    std::shared_ptr<MathematicOperable> num0(new ConstantNumber(0));
    std::shared_ptr<MathematicOperable> num0_1(new ConstantNumber(0.1));
    std::shared_ptr<MathematicOperable> num600(new ConstantNumber(600));
    std::shared_ptr<MathematicOperable> num2(new ConstantNumber(0.00033));

    ProtocolGraph::ProtocolNodePtr op1 = std::make_shared<AssignationOperation>(serial.getNextValue(),
        epsilon, num0_1); //epsilon = 1
    protocol->setStartNode(op1->getContainerId());

    ProtocolGraph::ProtocolNodePtr op2 = std::make_shared<AssignationOperation>(serial.getNextValue(),
        threshold, num600); //threshold = 600
    ProtocolGraph::ProtocolNodePtr op3 = std::make_shared<AssignationOperation>(serial.getNextValue(), rate,
        num2); //rate = 2

    protocol->addOperation(op1);
    protocol->addOperation(op2);
    protocol->addOperation(op3);

    std::shared_ptr<ComparisonOperable> tautology(new Tautology());
    protocol->connectOperation(op1, op2, tautology);
    protocol->connectOperation(op2, op3, tautology);

    std::shared_ptr<MathematicOperable> num1000(new ConstantNumber(100));
    ProtocolGraph::ProtocolNodePtr op4 = std::make_shared<LoadContainerOperation>(serial.getNextValue(), 1, num1000); //loadContainer(1, 1000ml)
    ProtocolGraph::ProtocolNodePtr op5 = std::make_shared<LoadContainerOperation>(serial.getNextValue(), 2, num0); //loadContainer(2, 1000ml)
    ProtocolGraph::ProtocolNodePtr op6 = std::make_shared<LoadContainerOperation>(serial.getNextValue(), 3, num0); //loadContainer(3, 1000ml)

    protocol->addOperation(op4);
    protocol->addOperation(op5);
    protocol->addOperation(op6);

    protocol->connectOperation(op3, op4, tautology);
    protocol->connectOperation(op4, op5, tautology);
    protocol->connectOperation(op5, op6, tautology);

    std::shared_ptr<MathematicOperable> num20(new ConstantNumber(20000));
    std::shared_ptr<VariableEntry> time(
        new VariableEntry(TIME_VARIABLE));
    std::shared_ptr<MathematicOperable> mtime(
        new VariableEntry(TIME_VARIABLE));

    std::shared_ptr<ComparisonOperable> comp1(
        new SimpleComparison(false, mtime, comparison::less, num20));

    ProtocolGraph::ProtocolNodePtr loop1 = std::make_shared<LoopNode>(serial.getNextValue(), comp1); //while (t < 20)

    protocol->addOperation(loop1);
    protocol->connectOperation(op6, loop1, tautology);

    std::shared_ptr<VariableEntry> od(new VariableEntry("od"));
    std::shared_ptr<MathematicOperable> mod(new VariableEntry("od"));
    ProtocolGraph::ProtocolNodePtr op7 = std::make_shared<MeasureOD>(serial.getNextValue(), 2, od); //od = measureOd(2)

    protocol->addOperation(op7);

    ProtocolGraph::ProtocolNodePtr timeStep = std::make_shared<TimeStep>(serial.getNextValue(), time);
    protocol->addOperation(timeStep);

    shared_ptr<OperationNode> nodeIf = createifOperation(0, 20000, op7, timeStep, serial, protocol);

    protocol->connectOperation(loop1,nodeIf, comp1);

    std::shared_ptr<MathematicOperable> num1(new ConstantNumber(1.0));
    std::shared_ptr<VariableEntry> normOD(new VariableEntry("normOD"));
    std::shared_ptr<MathematicOperable> mnormOD(
        new VariableEntry("normOD"));

    std::shared_ptr<MathematicOperable> operation1_1( //(od - threshold)
        new ArithmeticOperation(threshold, arithmetic::minus, od));
    std::shared_ptr<MathematicOperable> operation1( //(od - threshold) /threshold
        new ArithmeticOperation(operation1_1, arithmetic::divide,
            mthreshold));

    ProtocolGraph::ProtocolNodePtr op8 = std::make_shared<AssignationOperation>(serial.getNextValue(), normOD,
        operation1); // normOD = (od - threshold) /threshold

    protocol->addOperation(op8);
    protocol->connectOperation(op7, op8, tautology);

    shared_ptr<MathematicOperable> normOdPlusRate =
            shared_ptr<MathematicOperable>(new ArithmeticOperation(normOD, arithmetic::multiply, rate)); //normOd*rate
    shared_ptr<MathematicOperable> normOdPlusRate2 =
            shared_ptr<MathematicOperable>(new ArithmeticOperation(rate, arithmetic::plus, normOdPlusRate)); //rate + normOd*rate
    ProtocolGraph::ProtocolNodePtr op9 = std::make_shared<AssignationOperation>(serial.getNextValue(), rate,
        normOdPlusRate2); // rate = rate + prop*rate

    protocol->addOperation(op9);
    protocol->connectOperation(op8, op9, tautology);

    ProtocolGraph::ProtocolNodePtr op10 = std::make_shared<SetContinousFlow>(serial.getNextValue(), 1, 2,
        rate); // setcontinousFlow(1,2,rate)

    protocol->addOperation(op10);
    protocol->connectOperation(op9, op10, tautology);

    ProtocolGraph::ProtocolNodePtr op11 = std::make_shared<SetContinousFlow>(serial.getNextValue(), 2, 3,
        rate); // setcontinousFlow(2,3,rate)
    protocol->addOperation(op11);
    protocol->connectOperation(op10, op11, tautology);

    protocol->connectOperation(op11, timeStep, tautology);
    protocol->connectOperation(timeStep, loop1, tautology);
    return protocol;
}

ProtocolGraph* makeQuemostat() {
    AutoEnumerate serial;
    ProtocolGraph* protocol = new ProtocolGraph("quemostat");

    std::shared_ptr<MathematicOperable> num0(new ConstantNumber(0));
    std::shared_ptr<MathematicOperable> num2(new ConstantNumber(0.00033));
    std::shared_ptr<MathematicOperable> num1000(new ConstantNumber(100));

    std::shared_ptr<ComparisonOperable> tautology(new Tautology());

    ProtocolGraph::ProtocolNodePtr op4 = std::make_shared<LoadContainerOperation>(serial.getNextValue(), 1, num1000); //loadContainer(1, 1000ml)
    ProtocolGraph::ProtocolNodePtr op5 = std::make_shared<LoadContainerOperation>(serial.getNextValue(), 2, num0); //loadContainer(2, 1000ml)
    ProtocolGraph::ProtocolNodePtr op6 = std::make_shared<LoadContainerOperation>(serial.getNextValue(), 3, num0); //loadContainer(3, 1000ml)

    protocol->addOperation(op4);
    protocol->addOperation(op5);
    protocol->addOperation(op6);

    protocol->setStartNode(0);
    protocol->connectOperation(op4, op5, tautology);
    protocol->connectOperation(op5, op6, tautology);

    std::shared_ptr<MathematicOperable> num20(new ConstantNumber(20000));
    std::shared_ptr<VariableEntry> time(
        new VariableEntry(TIME_VARIABLE));
    std::shared_ptr<MathematicOperable> mtime(
        new VariableEntry(TIME_VARIABLE));

    std::shared_ptr<ComparisonOperable> comp1(
        new SimpleComparison(false, mtime, comparison::less, num20));

    ProtocolGraph::ProtocolNodePtr loop1 = std::make_shared<LoopNode>(serial.getNextValue(), comp1); //while (t < 20)

    protocol->addOperation(loop1);
    protocol->connectOperation(op6, loop1, tautology);

    ProtocolGraph::ProtocolNodePtr timeStep = std::make_shared<TimeStep>(serial.getNextValue(), time);
    protocol->addOperation(timeStep);

    ProtocolGraph::ProtocolNodePtr op10 = std::make_shared<SetContinousFlow>(serial.getNextValue(), 1, 2,
        num2); // setcontinousFlow(1,2,rate)

    protocol->addOperation(op10);
    shared_ptr<OperationNode> nodeIf = createifOperation(0, 20000, op10, timeStep, serial, protocol);

    protocol->connectOperation(loop1,nodeIf, comp1);

    ProtocolGraph::ProtocolNodePtr op11 = std::make_shared<SetContinousFlow>(serial.getNextValue(), 2, 3,
        num2); // setcontinousFlow(2,3,rate)
    protocol->addOperation(op11);
    protocol->connectOperation(op10, op11, tautology);

    protocol->connectOperation(op11, timeStep, tautology);
    protocol->connectOperation(timeStep, loop1, tautology);
    return protocol;
}

ProtocolGraph* makeEvoprogProtocolClean() {
    ProtocolGraph* protocol = new ProtocolGraph("evoprogProtocol");
    AutoEnumerate serial;
    protocol->setStartNode(0);

    std::shared_ptr<ComparisonOperable> tautology(new Tautology());
    std::shared_ptr<MathematicOperable> num0(new ConstantNumber(0));
    std::shared_ptr<MathematicOperable> num1000(new ConstantNumber(100));
    std::shared_ptr<MathematicOperable> num2(new ConstantNumber(0.02));

    ProtocolGraph::ProtocolNodePtr op0 = std::make_shared<LoadContainerOperation>(serial.getNextValue(), MEDIA_1, num1000); //loadContainer(1, 1000ml)
    ProtocolGraph::ProtocolNodePtr op1 = std::make_shared<LoadContainerOperation>(serial.getNextValue(), MEDIA_2, num1000); //loadContainer(2, 1000ml)
    ProtocolGraph::ProtocolNodePtr op2 = std::make_shared<LoadContainerOperation>(serial.getNextValue(), NaOH, num1000); //loadContainer(3, 1000ml)
    ProtocolGraph::ProtocolNodePtr op3 = std::make_shared<LoadContainerOperation>(serial.getNextValue(), WATER, num1000); //loadContainer(1, 1000ml)
    ProtocolGraph::ProtocolNodePtr op4 = std::make_shared<LoadContainerOperation>(serial.getNextValue(), AIR, num1000); //loadContainer(2, 1000ml)
    ProtocolGraph::ProtocolNodePtr op5 = std::make_shared<LoadContainerOperation>(serial.getNextValue(), ETHANOL, num1000); //loadContainer(3, 1000ml)
    ProtocolGraph::ProtocolNodePtr op6 = std::make_shared<LoadContainerOperation>(serial.getNextValue(), CHEMO_1, num0); //loadContainer(1, 1000ml)
    ProtocolGraph::ProtocolNodePtr op7 = std::make_shared<LoadContainerOperation>(serial.getNextValue(), CHEMO_2, num0); //loadContainer(2, 1000ml)
    ProtocolGraph::ProtocolNodePtr op8 = std::make_shared<LoadContainerOperation>(serial.getNextValue(), CELLSTAT, num0); //loadContainer(3, 1000ml)
    ProtocolGraph::ProtocolNodePtr op9 = std::make_shared<LoadContainerOperation>(serial.getNextValue(), WASTE, num0); //loadContainer(1, 1000ml)
    ProtocolGraph::ProtocolNodePtr op10 = std::make_shared<LoadContainerOperation>(serial.getNextValue(), CLEANING_WASTE, num0); //loadContainer(2, 1000ml)

    protocol->addOperation(op0);
    protocol->addOperation(op1);
    protocol->addOperation(op2);
    protocol->addOperation(op3);
    protocol->addOperation(op4);
    protocol->addOperation(op5);
    protocol->addOperation(op6);
    protocol->addOperation(op7);
    protocol->addOperation(op8);
    protocol->addOperation(op9);
    protocol->addOperation(op10);

    protocol->connectOperation(op0, op1, tautology);
    protocol->connectOperation(op1, op2, tautology);
    protocol->connectOperation(op2, op3, tautology);
    protocol->connectOperation(op3, op4, tautology);
    protocol->connectOperation(op4, op5, tautology);
    protocol->connectOperation(op5, op6, tautology);
    protocol->connectOperation(op6, op7, tautology);
    protocol->connectOperation(op7, op8, tautology);
    protocol->connectOperation(op8, op9, tautology);
    protocol->connectOperation(op9, op10, tautology);

    std::shared_ptr<MathematicOperable> num20(new ConstantNumber(180000));
    std::shared_ptr<VariableEntry> time(
        new VariableEntry(TIME_VARIABLE));
    std::shared_ptr<MathematicOperable> mtime(
        new VariableEntry(TIME_VARIABLE));
    std::shared_ptr<ComparisonOperable> comp1(
        new SimpleComparison(false, mtime, comparison::less, num20));

    ProtocolGraph::ProtocolNodePtr loop1 = std::make_shared<LoopNode>(serial.getNextValue(), comp1);

    protocol->addOperation(loop1);
    protocol->connectOperation(op10, loop1, tautology);

    int actualTime = 0;
    ProtocolGraph::ProtocolNodePtr lastNodeBefore = loop1;
    ProtocolGraph::ProtocolNodePtr lastIf;
    shared_ptr<ComparisonOperable> lastNotComparison;

    //step 1
    for (int i = 0; i < 3; i++) {

        ProtocolGraph::ProtocolNodePtr opLChemo1 = std::make_shared<SetContinousFlow>(serial.getNextValue(), NaOH + i, CHEMO_1, num2);
        ProtocolGraph::ProtocolNodePtr opChemo1Cell = std::make_shared<SetContinousFlow>(serial.getNextValue(), CHEMO_1, CELLSTAT, num2);
        ProtocolGraph::ProtocolNodePtr opCellWaste = std::make_shared<SetContinousFlow>(serial.getNextValue(), CELLSTAT, WASTE, num2);

        protocol->addOperation(opLChemo1);
        protocol->addOperation(opChemo1Cell);
        protocol->addOperation(opCellWaste);

        protocol->connectOperation(opLChemo1, opChemo1Cell, tautology);
        protocol->connectOperation(opChemo1Cell, opCellWaste, tautology);

        shared_ptr<ComparisonOperable> tempComp;
        ProtocolGraph::ProtocolNodePtr nodeIf1 = createifOperation(actualTime, 5000, opLChemo1, NULL, serial, protocol, tempComp);

        if (lastIf) {
            protocol->connectOperation(lastIf, nodeIf1, lastNotComparison);
        }

        if (i == 0) {
            protocol->connectOperation(lastNodeBefore, nodeIf1, comp1);
        } else {
            protocol->connectOperation(lastNodeBefore, nodeIf1, tautology);
        }

        lastNodeBefore = opCellWaste;
        lastIf = nodeIf1;
        lastNotComparison = tempComp;
        actualTime += 5000;

        ProtocolGraph::ProtocolNodePtr opLChemo2 = std::make_shared<SetContinousFlow>(serial.getNextValue(), NaOH + i, CHEMO_2, num2);
        ProtocolGraph::ProtocolNodePtr opChemo2Cell = std::make_shared<SetContinousFlow>(serial.getNextValue(), CHEMO_2, CELLSTAT, num2);
        ProtocolGraph::ProtocolNodePtr opCellWaste2 = std::make_shared<SetContinousFlow>(serial.getNextValue(), CELLSTAT, WASTE, num2);

        protocol->addOperation(opLChemo2);
        protocol->addOperation(opChemo2Cell);
        protocol->addOperation(opCellWaste2);

        protocol->connectOperation(opLChemo2, opChemo2Cell, tautology);

        protocol->connectOperation(opChemo2Cell, opCellWaste2, tautology);

        shared_ptr<ComparisonOperable> tempComp2;
        ProtocolGraph::ProtocolNodePtr nodeIf12 = createifOperation(actualTime, 5000, opLChemo2, NULL, serial, protocol, tempComp2);

        if (lastIf) {
            protocol->connectOperation(lastIf, nodeIf12, lastNotComparison);
        }

        protocol->connectOperation(lastNodeBefore, nodeIf12, tautology);

        lastNodeBefore = opCellWaste2;
        lastIf = nodeIf12;
        lastNotComparison = tempComp2;
        actualTime += 5000;
    }

    for (int i = 0; i < 3; i++) {
        ProtocolGraph::ProtocolNodePtr opLChemo1 = std::make_shared<SetContinousFlow>(serial.getNextValue(), NaOH + i, CHEMO_2, num2);
        ProtocolGraph::ProtocolNodePtr opChemo1Cell = std::make_shared<SetContinousFlow>(serial.getNextValue(), CHEMO_2, CELLSTAT, num2);
        ProtocolGraph::ProtocolNodePtr opCellWaste = std::make_shared<SetContinousFlow>(serial.getNextValue(), CELLSTAT, CLEANING_WASTE, num2);

        protocol->addOperation(opLChemo1);
        protocol->addOperation(opChemo1Cell);
        protocol->addOperation(opCellWaste);

        protocol->connectOperation(opLChemo1, opChemo1Cell, tautology);
        protocol->connectOperation(opChemo1Cell, opCellWaste, tautology);

        shared_ptr<ComparisonOperable> tempComp;
        ProtocolGraph::ProtocolNodePtr nodeIf1 = createifOperation(actualTime, 5000, opLChemo1, NULL, serial, protocol, tempComp);

        if (lastIf) {
            protocol->connectOperation(lastIf, nodeIf1, lastNotComparison);
        }

        protocol->connectOperation(lastNodeBefore, nodeIf1, tautology);

        lastNodeBefore = opCellWaste;
        lastIf = nodeIf1;
        lastNotComparison = tempComp;
        actualTime += 5000;
    }

    //step 2
    for (int i = 0; i < 3; i++) {
        ProtocolGraph::ProtocolNodePtr opLChemo1 = std::make_shared<SetContinousFlow>(serial.getNextValue(), NaOH + i, CHEMO_1, num2);
        ProtocolGraph::ProtocolNodePtr opChemo1Cell = std::make_shared<SetContinousFlow>(serial.getNextValue(), CHEMO_1, CLEANING_WASTE, num2);

        protocol->addOperation(opLChemo1);
        protocol->addOperation(opChemo1Cell);

        protocol->connectOperation(opLChemo1, opChemo1Cell, tautology);

        shared_ptr<ComparisonOperable> tempComp;
        ProtocolGraph::ProtocolNodePtr nodeIf1 = createifOperation(actualTime, 5000, opLChemo1, NULL, serial, protocol, tempComp);

        if (lastIf) {
            protocol->connectOperation(lastIf, nodeIf1, lastNotComparison);
        }

        protocol->connectOperation(lastNodeBefore, nodeIf1, tautology);

        lastNodeBefore = opChemo1Cell;
        lastIf = nodeIf1;
        lastNotComparison = tempComp;
        actualTime += 5000;

        ProtocolGraph::ProtocolNodePtr opLChemo2 = std::make_shared<SetContinousFlow>(serial.getNextValue(), NaOH + i, CHEMO_2, num2);
        ProtocolGraph::ProtocolNodePtr opChemo2Cell = std::make_shared<SetContinousFlow>(serial.getNextValue(), CHEMO_2, CLEANING_WASTE, num2);

        protocol->addOperation(opLChemo2);
        protocol->addOperation(opChemo2Cell);

        protocol->connectOperation(opLChemo2, opChemo2Cell, tautology);

        shared_ptr<ComparisonOperable> tempComp2;
        ProtocolGraph::ProtocolNodePtr nodeIf12 = createifOperation(actualTime, 5000, opLChemo2, NULL, serial, protocol, tempComp2);

        if (lastIf) {
            protocol->connectOperation(lastIf, nodeIf12, lastNotComparison);
        }

        protocol->connectOperation(lastNodeBefore, nodeIf12, tautology);

        lastNodeBefore = opChemo2Cell;
        lastIf = nodeIf12;
        lastNotComparison = tempComp2;
        actualTime += 5000;
    }

    //step 3
    for (int i = 0; i < 2; i++) {
        ProtocolGraph::ProtocolNodePtr opLChemo1 = std::make_shared<SetContinousFlow>(serial.getNextValue(), AIR, CHEMO_1 + i, num2);
        ProtocolGraph::ProtocolNodePtr opChemo1Cell = std::make_shared<SetContinousFlow>(serial.getNextValue(), CHEMO_1 + i, CLEANING_WASTE, num2);

        protocol->addOperation(opLChemo1);
        protocol->addOperation(opChemo1Cell);

        protocol->connectOperation(opLChemo1, opChemo1Cell, tautology);

        shared_ptr<ComparisonOperable> tempComp;
        ProtocolGraph::ProtocolNodePtr nodeIf1 = createifOperation(actualTime, 25000, opLChemo1, NULL, serial, protocol, tempComp);

        if (lastIf) {
            protocol->connectOperation(lastIf, nodeIf1, lastNotComparison);
        }

        protocol->connectOperation(lastNodeBefore, nodeIf1, tautology);

        lastNodeBefore = opChemo1Cell;
        lastIf = nodeIf1;
        lastNotComparison = tempComp;
        actualTime += 25000;

        ProtocolGraph::ProtocolNodePtr stop = std::make_shared<SetContinousFlow>(serial.getNextValue(), AIR, CHEMO_1 + i, num0);
        ProtocolGraph::ProtocolNodePtr opLChemo2 = std::make_shared<SetContinousFlow>(serial.getNextValue(), MEDIA_1 + i, CHEMO_1 + i, num2);
        ProtocolGraph::ProtocolNodePtr opChemo2Cell = std::make_shared<SetContinousFlow>(serial.getNextValue(), CHEMO_1 + i, CELLSTAT, num2);
        ProtocolGraph::ProtocolNodePtr opCellWaste = std::make_shared<SetContinousFlow>(serial.getNextValue(), CELLSTAT, WASTE, num2);

        protocol->addOperation(stop);
        protocol->addOperation(opLChemo2);
        protocol->addOperation(opChemo2Cell);
        protocol->addOperation(opCellWaste);

        protocol->connectOperation(stop, opLChemo2, tautology);
        protocol->connectOperation(opLChemo2, opChemo2Cell, tautology);
        protocol->connectOperation(opChemo2Cell, opCellWaste, tautology);

        shared_ptr<ComparisonOperable> tempComp2;
        ProtocolGraph::ProtocolNodePtr nodeIf12 = createifOperation(actualTime, 25000, stop, NULL, serial, protocol, tempComp2);

        if (lastIf) {
            protocol->connectOperation(lastIf, nodeIf12, lastNotComparison);
        }

        protocol->connectOperation(lastNodeBefore, nodeIf12, tautology);

        lastNodeBefore = opCellWaste;
        lastIf = nodeIf12;
        lastNotComparison = tempComp2;
        actualTime += 25000;
    }

    //step 4
    ProtocolGraph::ProtocolNodePtr opLChemo1 = std::make_shared<SetContinousFlow>(serial.getNextValue(), MEDIA_2, CHEMO_2, num0);
    protocol->addOperation(opLChemo1);

    shared_ptr<ComparisonOperable> tempComp;
    ProtocolGraph::ProtocolNodePtr nodeIf1 = createifOperation(actualTime, 1000, opLChemo1, NULL, serial, protocol, tempComp);

    if (lastIf) {
        protocol->connectOperation(lastIf, nodeIf1, lastNotComparison);
    }

    protocol->connectOperation(lastNodeBefore, nodeIf1, tautology);

    lastNodeBefore = opLChemo1;
    lastIf = nodeIf1;
    lastNotComparison = tempComp;
    actualTime += 1000;

    //loop

    ProtocolGraph::ProtocolNodePtr timeStep = std::make_shared<TimeStep>(serial.getNextValue(), time);

    protocol->addOperation(timeStep);
    protocol->connectOperation(lastNodeBefore, timeStep, tautology);
    protocol->connectOperation(lastIf, timeStep, lastNotComparison);
    protocol->connectOperation(timeStep, loop1, tautology);

    return protocol;
}

void testPathManager() {
    LOG(INFO) << "creating machine...";
    std::shared_ptr<ExecutableMachineGraph> machine = std::shared_ptr<ExecutableMachineGraph>(ExecutableMachineGraph::fromJSON("machineLayoutv3.json"));

    PathManager manager(machine);

    LOG(INFO) << " flows from inlet to bidirectional..";

    shared_ptr<SearcherIterator> it = manager.getFlows(make_shared<ContainerNodeType>(MovementType::continuous, ContainerType::inlet), make_shared<ContainerNodeType>(MovementType::continuous, ContainerType::bidirectional_switch));

    while (it->hasNext()) {
        LOG(INFO) << it->next()->toText();
    }
}

MachineGraph* makeTurbidostatSketch() {
    MachineGraph* sketch = new MachineGraph("sketchTurbidostat");

    std::shared_ptr<ContainerNodeType> cinlet(new ContainerNodeType(MovementType::continuous, ContainerType::inlet));
    std::shared_ptr<ContainerNodeType> cFlow(new ContainerNodeType(MovementType::continuous, ContainerType::flow));
    std::shared_ptr<ContainerNodeType> sink(new ContainerNodeType(MovementType::irrelevant, ContainerType::sink));

    sketch->addContainer(1, cinlet, 100.0);
    sketch->addContainer(2, cFlow, 100.0);
    sketch->addContainer(3, sink, 100.0);

    sketch->connectContainer(1, 2);
    sketch->connectContainer(2, 3);

    return sketch;
}

ProtocolGraph* makeTimeProtocol()
{
    AutoEnumerate serial;
    ProtocolGraph* protocol = new ProtocolGraph("simpleProtocol");

    std::shared_ptr<ComparisonOperable> tautology(new Tautology());
    std::shared_ptr<MathematicOperable> num1(new ConstantNumber(0.001));
    std::shared_ptr<MathematicOperable> num60000(new ConstantNumber(60000));
    std::shared_ptr<MathematicOperable> num65(new ConstantNumber(65));

    ProtocolGraph::ProtocolNodePtr op1 = std::make_shared<LoadContainerOperation>(serial.getNextValue(), 1, num65); //loadContainer(1, 1000ml)

    protocol->addOperation(op1);

    std::shared_ptr<VariableEntry> time(
        new VariableEntry(TIME_VARIABLE));
    std::shared_ptr<MathematicOperable> mtime(
        new VariableEntry(TIME_VARIABLE));
    std::shared_ptr<ComparisonOperable> comp2in(
        new SimpleComparison(false, mtime, comparison::less_equal, num60000));
    ProtocolGraph::ProtocolNodePtr loop1 = std::make_shared<LoopNode>(serial.getNextValue(), comp2in); //while ( t <= 60s)

    protocol->addOperation(loop1);
    protocol->connectOperation(op1, loop1, tautology);

    ProtocolGraph::ProtocolNodePtr op2 = std::make_shared<SetContinousFlow>(serial.getNextValue(), 1, 2, num1);
    ProtocolGraph::ProtocolNodePtr op3 = std::make_shared<SetContinousFlow>(serial.getNextValue(), 2, 3, num1);

    protocol->addOperation(op2);
    protocol->connectOperation(loop1, op2, comp2in);
    protocol->addOperation(op3);
    protocol->connectOperation(op2, op3, tautology);
    ProtocolGraph::ProtocolNodePtr timeStep = std::make_shared<TimeStep>(serial.getNextValue(), time);

    protocol->addOperation(timeStep);
    protocol->connectOperation(op3, timeStep, tautology);
    protocol->connectOperation(timeStep, loop1, tautology);

    protocol->setStartNode(op1->getContainerId());
    return protocol;
}

ExecutableMachineGraph* makeMappingMachine(int communications,
                                                      std::unique_ptr<CommandSender> exec,
                                                      std::unique_ptr<CommandSender> test)
{
    ExecutableMachineGraph* machine = new ExecutableMachineGraph(
                "mappingMachine", std::move(exec), std::move(test));

    std::unordered_map<std::string, std::string> paramsc = {{"address","46"},
                                                            {"closePos","0"}};
    std::shared_ptr<Control> control(
                new ControlPlugin(communications,"v1", "Evoprog4WayValve", paramsc));

    std::unordered_map<std::string, std::string> paramse = {{"address","7"},
                                                            {"direction","0"}};
    std::shared_ptr<Extractor> cExtractor(
                new ExtractorPlugin(communications,"p1", "EvoprogV2Pump", paramse));

    std::unordered_map<std::string, std::string> paramsi;
    std::shared_ptr<Injector> dummyInjector(
                new InjectorPlugin(communications, "dummy", "EvoprogDummyInjector", paramsi));

    std::unordered_map<std::string, std::string> paramso = {{"pinNumber","14"}};
    std::shared_ptr<ODSensor> sensor(new ODSensorPlugin(communications, "sensor1", "EvoprogOdSensor", paramso));

    ExecutableMachineGraph::ExecutableContainerNodePtr cInlet1 = std::make_shared<InletContainer>(1, 100.0, cExtractor);
    ExecutableMachineGraph::ExecutableContainerNodePtr cInlet2 = std::make_shared<DivergentSwitchSink>(2, 100.0, dummyInjector, cExtractor, control);
    ExecutableMachineGraph::ExecutableContainerNodePtr cInlet3 = std::make_shared<FlowContainer>(3, 100.0, cExtractor, dummyInjector);
    ExecutableMachineGraph::ExecutableContainerNodePtr cInlet4 = std::make_shared<InletContainer>(4, 100.0, cExtractor);


    ExecutableMachineGraph::ExecutableContainerNodePtr cSwtInlet5 = std::make_shared<ConvergentSwitchInlet>(5, 100.0,
                                                                                                            dummyInjector, cExtractor, control);
    ExecutableMachineGraph::ExecutableContainerNodePtr cSwtInlet6 = std::make_shared<BidirectionalSwitch>(6, 100.0,
                                                                                                          cExtractor, dummyInjector, control, control);
    cSwtInlet6->setOd(sensor);
    ExecutableMachineGraph::ExecutableContainerNodePtr cSwich7 = std::make_shared<ConvergentSwitch>(7, 100.0, dummyInjector, control);

    machine->addContainer(cInlet1);
    machine->addContainer(cInlet2);
    machine->addContainer(cInlet3);
    machine->addContainer(cInlet4);
    machine->addContainer(cSwtInlet5);
    machine->addContainer(cSwtInlet6);
    machine->addContainer(cSwich7);

    machine->connectExecutableContainer(1, 5);
    machine->connectExecutableContainer(2, 5);
    machine->connectExecutableContainer(3, 6);
    machine->connectExecutableContainer(4, 6);
    machine->connectExecutableContainer(5, 7);
    machine->connectExecutableContainer(6, 7);
    machine->connectExecutableContainer(6, 2);
    machine->connectExecutableContainer(2, 3);

    return machine;
}

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    el::Helpers::setStorage(sharedLoggingRepository());

    el::Configurations c("./configuration/log.ini");
    el::Loggers::reconfigureAllLoggers(c);

    /*LOG(INFO) << "making turbidostat...";
    ProtocolGraph* protocol = makeTurbidostat();
    LOG(INFO) << "printing graph...";
    protocol->printProtocol("turbidostatProtocol.graph");
    LOG(INFO) << "printing JSON...";
    ProtocolGraph::toJSON("turbidostatProtocol.json", *protocol);

    LOG(INFO) << "making chemostat...";
    ProtocolGraph* chemo = makeQuemostat();
    LOG(INFO) << "printing graph...";
    chemo->printProtocol("chemostatProtocol.graph");
    LOG(INFO) << "printing JSON...";
    ProtocolGraph::toJSON("chemostatProtocol.json", *chemo);*/

    /*LOG(INFO) << "making cleaning...";
    ProtocolGraph* chemo = makeEvoprogProtocolClean();
    LOG(INFO) << "printing graph...";
    chemo->printProtocol("cleaningProtocol.graph");
    LOG(INFO) << "printing JSON...";
    ProtocolGraph::toJSON("cleaningProtocol.json", *chemo);*/

    //testPathManager();

    /*LOG(INFO) << "making turbidostat sketch...";
    MachineGraph* chemo = makeTurbidostatSketch();
    LOG(INFO) << "printing graph...";
    chemo->printMachine("turbidostatSketch.graph");
    LOG(INFO) << "generating JSON...";
    MachineGraph::toJSON("turbidostatSketch.json", *chemo);*/

    /*LOG(INFO) << "making protocol...";
    ProtocolGraph* chemo = makeTimeProtocol();
    LOG(INFO) << "printing graph...";
    chemo->printProtocol("timeProtocol.graph");
    LOG(INFO) << "generating JSON...";
    ProtocolGraph::toJSON("timeProtocol.json", *chemo);*/

    /*PythonEnvironment::GetInstance()->initEnvironment();

    std::unique_ptr<CommandSender> comEx = std::unique_ptr<CommandSender>(new SerialSender("\\\\.\\COM3"));
    std::unique_ptr<CommandSender> comTest = std::unique_ptr<CommandSender>(new FileSender("test.log", "inputFileData.txt"));
    int com = CommunicationsInterface::GetInstance()->addCommandSender(comEx->clone());

    std::shared_ptr<ExecutableMachineGraph> machine(makeMappingMachine(com, std::move(comEx), std::move(comTest)));
    ExecutableMachineGraph::toJSON("exMachine.json", *machine.get());

    LOG(INFO) << "done!";

    PythonEnvironment::GetInstance()->finishEnvironment();*/
}













