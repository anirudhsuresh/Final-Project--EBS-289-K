function [T, oldT] = create_transform_matrix(q_next)
    trnsl = transl2(q_next(1), q_next(2));
    rotat = trot2(q_next(3));
    T = trnsl * rotat;
end