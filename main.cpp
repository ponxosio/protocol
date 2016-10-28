#define END_TIME 5000

#include <QCoreApplication>

#include <easylogging++.h>

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
    std::shared_ptr<MathematicOperable> num2(new ConstantNumber(2));

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
        new ArithmeticOperation(mod, arithmetic::minus, mthreshold));
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
    ProtocolGraph::ProtocolNodePtr op9 = std::make_shared<AssignationOperation>(serial.getNextValue(), normOD,
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

ProtocolGraph* makeEvoprogProtocol() {
   /* ProtocolGraph* protocol = new ProtocolGraph("evoprogProtocol");
    AutoEnumerate serial;
    protocol->setStartNode(0);

    std::shared_ptr<MathematicOperable> num1000(new ConstantNumber(100));
    ProtocolGraph::ProtocolNodePtr op0 = std::make_shared<LoadContainerOperation>(serial.getNextValue(), 1, num1000); //loadContainer(1, 1000ml)
    ProtocolGraph::ProtocolNodePtr op1 = std::make_shared<LoadContainerOperation>(serial.getNextValue(), 2, num0); //loadContainer(2, 1000ml)
    ProtocolGraph::ProtocolNodePtr op2 = std::make_shared<LoadContainerOperation>(serial.getNextValue(), 3, num0); //loadContainer(3, 1000ml)
    ProtocolGraph::ProtocolNodePtr op3 = std::make_shared<LoadContainerOperation>(serial.getNextValue(), 1, num1000); //loadContainer(1, 1000ml)
    ProtocolGraph::ProtocolNodePtr op4 = std::make_shared<LoadContainerOperation>(serial.getNextValue(), 2, num0); //loadContainer(2, 1000ml)
    ProtocolGraph::ProtocolNodePtr op5 = std::make_shared<LoadContainerOperation>(serial.getNextValue(), 3, num0); //loadContainer(3, 1000ml)
    ProtocolGraph::ProtocolNodePtr op6 = std::make_shared<LoadContainerOperation>(serial.getNextValue(), 1, num1000); //loadContainer(1, 1000ml)
    ProtocolGraph::ProtocolNodePtr op7 = std::make_shared<LoadContainerOperation>(serial.getNextValue(), 2, num0); //loadContainer(2, 1000ml)
    ProtocolGraph::ProtocolNodePtr op8 = std::make_shared<LoadContainerOperation>(serial.getNextValue(), 3, num0); //loadContainer(3, 1000ml)
    ProtocolGraph::ProtocolNodePtr op9 = std::make_shared<LoadContainerOperation>(serial.getNextValue(), 1, num1000); //loadContainer(1, 1000ml)
    ProtocolGraph::ProtocolNodePtr op10 = std::make_shared<LoadContainerOperation>(serial.getNextValue(), 2, num0); //loadContainer(2, 1000ml)

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
    protocol->connectOperation(op3, op4, tautology);
    protocol->connectOperation(op4, op5, tautology);
    protocol->connectOperation(op5, op6, tautology);
    protocol->connectOperation(op6, op7, tautology);
    protocol->connectOperation(op7, op8, tautology);
    protocol->connectOperation(op8, op9, tautology);
    protocol->connectOperation(op9, op10, tautology);
    protocol->connectOperation(op10, op11, tautology);
    protocol->connectOperation(op11, loop, tautology);

    ProtocolGraph::ProtocolNodePtr loop1 = std::make_shared<LoopNode>(serial.getNextValue(), comp1);
    std::shared_ptr<MathematicOperable> num20(new ConstantNumber(20000));
    std::shared_ptr<VariableEntry> time(
        new VariableEntry(TIME_VARIABLE));
    std::shared_ptr<MathematicOperable> mtime(
        new VariableEntry(TIME_VARIABLE));

    std::shared_ptr<ComparisonOperable> comp1(
        new SimpleComparison(false, mtime, comparison::less, num20));

    return protocol;*/
    return NULL;
}


int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    el::Helpers::setStorage(sharedLoggingRepository());

    el::Configurations c("./configuration/log.ini");
    el::Loggers::reconfigureAllLoggers(c);

    LOG(INFO) << "making protocol...";
    ProtocolGraph* protocol = makeTurbidostat();
    LOG(INFO) << "printing graph...";
    protocol->printProtocol("turbidostatProtocol.graph");
    LOG(INFO) << "printing JSON...";
    ProtocolGraph::toJSON("turbidostatProtocol.json", *protocol);
    LOG(INFO) << "done!";

    return a.exec();
}













