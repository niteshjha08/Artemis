
# Artemis

[![Build Status](https://github.com/tanujthakkar/Artemis/actions/workflows/build_and_coveralls.yml/badge.svg)](https://github.com/tanujthakkar/Artemis/actions/workflows/build_and_coveralls.yml)
[![Coverage Status](https://coveralls.io/repos/github/tanujthakkar/Artemis/badge.svg?branch=master)](https://coveralls.io/github/tanujthakkar/Artemis?branch=master)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

Artemis is a warehouse inventory management robot, the package for which is written in C++. The package is responsible for navigation in a warehouse environment, identifying the cargo, pick and place of the cargo. The location of the cargo is not known to the robot ahead of time but is estimated using ArUco tags. The cargo locations are randomized for each run.

## Project Members
<ul>
    <li>Nitesh Jha (UID - 117525366)</li>
    <li>Tanuj Thakkar (UID - 117817539)</li>
</ul>

## Overview
In warehousing, putaway refers to the process of movement of goods from the staging area, where the goods are received from trucks, to optimal warehouse storage locations. This is considered to be one of the most labor-intensive and expensive task. If putaway is not optimally executed, it can disrupt the entire workflow, resulting in delayed order fulfilment, additional cost of labor, and many other problems. It is also important that the accuracy of putaway be high, so that the picking process, which refers to obtaining individual items from storage to processing stations, is time-efficient.

We propose an autonomous robot, Artemis, for this process. An autonomous robot is advantageous as it will reduce the total time to recieve and process orders. Artemis consists of planners which optimally planning its motion, and it accurately identifies cargo and places them at optimal storage locations.

Artemis is a ground vehicle with an integrated manipulator arm. It is able to move to locations ideal for picking and execute pick and place tasks accurately.

## Project Design

<ul>
    <li><a href='https://github.com/tanujthakkar/Artemis/blob/master/Proposal/Images/System%20Design.png' >System Design</a></li>
    <li><a href='https://github.com/tanujthakkar/Artemis/blob/master/UML/initial/Class%20Diagram.pdf' >UML Class Diagram</a></li>
    <li><a href='https://github.com/tanujthakkar/Artemis/blob/master/UML/initial/Activity%20Diagram.pdf' >UML Activity Diagram</a></li>
</ul>

## Project Development
<ul>
    <li>Agile Iterative Process (AIP)</li>
    <ul>
        <li><a href="https://docs.google.com/document/d/1bkmZ2cDq_XIwyUF8y27-2fWmr7rnjcfL4RgtLlhhcCk/edit?usp=sharing">Sprint Notes</a></li>
        <li><a href="https://docs.google.com/spreadsheets/d/1H1erZtKIsuct7jV8JUrlnFfDlr3VFeBFLwXscopR-oU/edit?usp=sharing">Planning Sheet</a></li>
    </ul>
    <li>Test Driven Development (TDD)</li>
    <ul>
        <li>Phase 1</li>
        <ul>
            <li>Tanuj Thakkar</li>
            <li>Nitesh Jha</li>
        </ul>
    </ul>
    <ul>
        <li>Phase 2</li>
        <ul>
            <li>Driver: Tanuj Thakkar</li>
            <li>Navigator: Nitesh Jha</li>
        </ul>
    </ul>
    <ul>
        <li>Phase 2</li>
        <ul>
            <li>Driver: Nitesh Jha</li>
            <li>Navigator: Tanuj Thakkar</li>
        </ul>
    </ul>
</ul>


## Additional Links
<ul>
    <li>Proposal</li>
    <ul>
        <li><a href='https://github.com/tanujthakkar/Artemis/blob/master/Proposal/ENPM808X___Project_Proposal.pdf' >PDF</a></li>
    </ul>
</ul>
