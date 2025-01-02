import { Request, Response, NextFunction } from 'express';
import { StatusCodes } from 'http-status-codes';
import { MissionController } from '../../../src/controllers/mission.controller';
import { MissionService } from '../../../src/services/mission.service';
import { MissionStatus } from '../../../src/constants/missionStatus';
import { RobotType } from '../../../src/constants/robotTypes';
import { ErrorCodes } from '../../../src/constants/errorCodes';

// Mock MissionService
jest.mock('../../../src/services/mission.service');

// Mock Express request/response objects
const mockRequest = () => {
    const req = {} as Request;
    req.body = {};
    req.params = {};
    req.query = {};
    return req;
};

const mockResponse = () => {
    const res = {} as Response;
    res.status = jest.fn().mockReturnThis();
    res.json = jest.fn().mockReturnThis();
    return res;
};

// Test data
const testMission = {
    id: 'AGM-M-000001',
    name: 'Test Mission',
    type: 'surveillance',
    status: MissionStatus.CREATED,
    assignedDevices: [
        { deviceId: 'AGM-D-000001', type: RobotType.AERIAL_DRONE }
    ],
    coverageArea: {
        type: 'Polygon',
        coordinates: [[[0, 0], [1, 0], [1, 1], [0, 1], [0, 0]]],
        properties: {}
    },
    startTime: new Date(Date.now() + 3600000),
    progress: 0,
    parameters: {
        altitude: 50,
        speed: 5,
        overlap: 30
    },
    metadata: {
        createdAt: new Date(),
        updatedAt: new Date(),
        createdBy: 'test-user',
        version: 1,
        priority: 1,
        dependencies: [],
        offline: false
    }
};

describe('MissionController', () => {
    let req: Request;
    let res: Response;
    let next: NextFunction;

    beforeEach(() => {
        req = mockRequest();
        res = mockResponse();
        next = jest.fn();
        jest.clearAllMocks();
    });

    describe('createMission', () => {
        it('should create a mission successfully', async () => {
            req.body = testMission;
            (MissionService.createMission as jest.Mock).mockResolvedValue(testMission);

            await MissionController.createMission(req, res, next);

            expect(MissionService.createMission).toHaveBeenCalledWith(testMission);
            expect(res.status).toHaveBeenCalledWith(StatusCodes.CREATED);
            expect(res.json).toHaveBeenCalledWith(testMission);
        });

        it('should handle validation errors', async () => {
            req.body = { ...testMission, name: '' };

            await MissionController.createMission(req, res, next);

            expect(res.status).toHaveBeenCalledWith(StatusCodes.BAD_REQUEST);
            expect(res.json).toHaveBeenCalledWith(expect.objectContaining({
                code: ErrorCodes.VALIDATION_ERROR
            }));
        });

        it('should handle service errors', async () => {
            req.body = testMission;
            const error = new Error('Service error');
            (MissionService.createMission as jest.Mock).mockRejectedValue(error);

            await MissionController.createMission(req, res, next);

            expect(next).toHaveBeenCalledWith(error);
        });
    });

    describe('getMission', () => {
        it('should retrieve a mission successfully', async () => {
            req.params.id = testMission.id;
            (MissionService.getMissionById as jest.Mock).mockResolvedValue(testMission);

            await MissionController.getMission(req, res, next);

            expect(MissionService.getMissionById).toHaveBeenCalledWith(testMission.id);
            expect(res.status).toHaveBeenCalledWith(StatusCodes.OK);
            expect(res.json).toHaveBeenCalledWith(testMission);
        });

        it('should handle non-existent mission', async () => {
            req.params.id = 'non-existent';
            (MissionService.getMissionById as jest.Mock).mockResolvedValue(null);

            await MissionController.getMission(req, res, next);

            expect(res.status).toHaveBeenCalledWith(StatusCodes.NOT_FOUND);
            expect(res.json).toHaveBeenCalledWith(expect.objectContaining({
                code: ErrorCodes.RESOURCE_NOT_FOUND
            }));
        });
    });

    describe('updateMissionStatus', () => {
        it('should update mission status successfully', async () => {
            req.params.id = testMission.id;
            req.body = { status: MissionStatus.IN_PROGRESS };
            const updatedMission = { ...testMission, status: MissionStatus.IN_PROGRESS };
            (MissionService.updateMissionStatus as jest.Mock).mockResolvedValue(updatedMission);

            await MissionController.updateMissionStatus(req, res, next);

            expect(MissionService.updateMissionStatus).toHaveBeenCalledWith(
                testMission.id,
                MissionStatus.IN_PROGRESS
            );
            expect(res.status).toHaveBeenCalledWith(StatusCodes.OK);
            expect(res.json).toHaveBeenCalledWith(updatedMission);
        });

        it('should handle invalid status transitions', async () => {
            req.params.id = testMission.id;
            req.body = { status: 'INVALID_STATUS' };

            await MissionController.updateMissionStatus(req, res, next);

            expect(res.status).toHaveBeenCalledWith(StatusCodes.BAD_REQUEST);
            expect(res.json).toHaveBeenCalledWith(expect.objectContaining({
                code: ErrorCodes.VALIDATION_ERROR
            }));
        });
    });

    describe('updateMissionProgress', () => {
        it('should update mission progress successfully', async () => {
            req.params.id = testMission.id;
            req.body = { progress: 50 };
            const updatedMission = { ...testMission, progress: 50 };
            (MissionService.updateMissionProgress as jest.Mock).mockResolvedValue(updatedMission);

            await MissionController.updateMissionProgress(req, res, next);

            expect(MissionService.updateMissionProgress).toHaveBeenCalledWith(testMission.id, 50);
            expect(res.status).toHaveBeenCalledWith(StatusCodes.OK);
            expect(res.json).toHaveBeenCalledWith(updatedMission);
        });

        it('should handle invalid progress values', async () => {
            req.params.id = testMission.id;
            req.body = { progress: 150 };

            await MissionController.updateMissionProgress(req, res, next);

            expect(res.status).toHaveBeenCalledWith(StatusCodes.BAD_REQUEST);
            expect(res.json).toHaveBeenCalledWith(expect.objectContaining({
                code: ErrorCodes.VALIDATION_ERROR
            }));
        });
    });

    describe('getActiveMissions', () => {
        it('should retrieve active missions with pagination', async () => {
            req.query = { limit: '10', offset: '0', type: 'surveillance' };
            const missions = [testMission];
            (MissionService.getActiveMissions as jest.Mock).mockResolvedValue({ missions, total: 1 });

            await MissionController.getActiveMissions(req, res, next);

            expect(MissionService.getActiveMissions).toHaveBeenCalledWith({
                type: 'surveillance',
                limit: 10,
                offset: 0
            });
            expect(res.status).toHaveBeenCalledWith(StatusCodes.OK);
            expect(res.json).toHaveBeenCalledWith({ missions, total: 1 });
        });

        it('should handle service errors', async () => {
            req.query = { limit: '10', offset: '0' };
            const error = new Error('Service error');
            (MissionService.getActiveMissions as jest.Mock).mockRejectedValue(error);

            await MissionController.getActiveMissions(req, res, next);

            expect(next).toHaveBeenCalledWith(error);
        });
    });

    describe('assignDevice', () => {
        it('should assign device to mission successfully', async () => {
            req.params.id = testMission.id;
            req.body = { deviceId: 'AGM-D-000002' };
            const updatedMission = {
                ...testMission,
                assignedDevices: [
                    ...testMission.assignedDevices,
                    { deviceId: 'AGM-D-000002', type: RobotType.GROUND_ROBOT }
                ]
            };
            (MissionService.assignDeviceToMission as jest.Mock).mockResolvedValue(updatedMission);

            await MissionController.assignDevice(req, res, next);

            expect(MissionService.assignDeviceToMission).toHaveBeenCalledWith(
                testMission.id,
                'AGM-D-000002'
            );
            expect(res.status).toHaveBeenCalledWith(StatusCodes.OK);
            expect(res.json).toHaveBeenCalledWith(updatedMission);
        });

        it('should handle device assignment errors', async () => {
            req.params.id = testMission.id;
            req.body = { deviceId: 'AGM-D-000002' };
            const error = new Error('Device already assigned');
            (MissionService.assignDeviceToMission as jest.Mock).mockRejectedValue(error);

            await MissionController.assignDevice(req, res, next);

            expect(next).toHaveBeenCalledWith(error);
        });
    });
});