function model = spinn3d_train_alpha_gov(DS, out_path, opts)
% dlnetwork 训练：MSE + 软物理（经 governor 不越 α*）
% 严格版：只对“基础特征”做零/常数列剔除，尾巴 6 维固定保留；
%        mu/sig 仅用训练集计算；把 keepBaseIdx 一并保存到 model.preproc。

    assert(isfield(DS,'X') && isfield(DS,'y'), 'DS 需要包含 X/y');
    X = double(DS.X);
    y = double(DS.y(:));
    alpha_min = getfield_def(DS,'alpha_min', 0);

    X(~isfinite(X))=0; y(~isfinite(y))=alpha_min;
    y = min(1.0, max(alpha_min, y));

    [N,Dtot] = size(X); 
    assert(N>0 && Dtot>=6, '训练数据为空或维度不足');

    % —— 只对基础特征做列压缩（尾巴 6 维强制保留）——
    baseDim = Dtot - 6;
    Xbase   = X(:,1:baseDim);
    Xtail6  = X(:,baseDim+1:end);       % [a_prev, BETA, AUP, ADN, Ts, s_norm]

    if isfield(DS,'keepBaseIdx') && ~isempty(DS.keepBaseIdx)
        keepBaseIdx = logical(DS.keepBaseIdx(:)).';
        assert(numel(keepBaseIdx)==baseDim, 'DS.keepBaseIdx 长度与基础特征维不一致');
    else
        v = var(Xbase,0,1);
        keepBaseIdx = (v > 1e-12);
    end

    Xc = [Xbase(:, keepBaseIdx), Xtail6];
    Din = size(Xc,2);

    % 划分训练/验证，用训练集统计 z‑score
    rng(0,'twister');
    if N < 64, Ni=N; Nv=0; else, Nv=floor(0.15*N); Ni=N-Nv; end
    ord = randperm(N); Itr = ord(1:Ni); Iva = ord(Ni+1:end);

    mu  = mean(Xc(Itr,:),1);
    sig = std( Xc(Itr,:),0,1 ); sig(sig<1e-12)=1;

    Ztr = (Xc(Itr,:) - mu)./sig;  ytr = y(Itr);
    Zva = (Xc(Iva,:) - mu)./sig;  yva = y(Iva);

    % 网络
    layers = [
        featureInputLayer(Din, Normalization="none", Name="in")
        fullyConnectedLayer(256, Name="fc1")
        reluLayer(Name="relu1")
        fullyConnectedLayer(128, Name="fc2")
        reluLayer(Name="relu2")
        fullyConnectedLayer(1,   Name="z")
        sigmoidLayer(Name="sig")
    ];
    net = dlnetwork(layerGraph(layers));

    % 超参
    if nargin<3, opts=struct(); end
    if ~isfield(opts,'MaxEpochs'),        opts.MaxEpochs = 40; end
    if ~isfield(opts,'MiniBatchSize'),    opts.MiniBatchSize = min(512, max(8, floor(Ni/8))); end
    if ~isfield(opts,'InitialLearnRate'), opts.InitialLearnRate = 1e-3; end
    if ~isfield(opts,'LambdaOver'),       opts.LambdaOver = 8.0; end
    if ~isfield(opts,'LambdaUnder'),      opts.LambdaUnder = 0.5; end
    epochs=opts.MaxEpochs; mbs=opts.MiniBatchSize; lr=opts.InitialLearnRate;
    lamO=opts.LambdaOver; lamU=opts.LambdaUnder;

    % 提前取出 governor 常量（未归一化，索引相对 Xc）
    idx_ap = Din-5; idx_be = Din-4; idx_up = Din-3; idx_dn = Din-2; idx_ts = Din-1;

    Ap_tr = Xc(Itr,idx_ap); Be_tr = Xc(Itr,idx_be); Up_tr = Xc(Itr,idx_up);
    Dn_tr = Xc(Itr,idx_dn); Ts_tr = Xc(Itr,idx_ts);

    Ap_va = Xc(Iva,idx_ap); Be_va = Xc(Iva,idx_be); Up_va = Xc(Iva,idx_up);
    Dn_va = Xc(Iva,idx_dn); Ts_va = Xc(Iva,idx_ts);

    avgGrad=[]; avgSqGrad=[]; it=0; Ni_per_epoch=ceil(Ni/mbs);
    for ep=1:epochs
        ord2 = randperm(Ni);
        for b=1:Ni_per_epoch
            it = it + 1;
            j1=(b-1)*mbs+1; j2=min(Ni,b*mbs); I = ord2(j1:j2);

            dlX = dlarray(single(Ztr(I,:)'), 'CB');
            dly = dlarray(single(ytr(I,:)'), 'CB');
            dlAp= dlarray(single(Ap_tr(I,:)'), 'CB');
            dlBe= dlarray(single(Be_tr(I,:)'), 'CB');
            dlUp= dlarray(single(Up_tr(I,:)'), 'CB');
            dlDn= dlarray(single(Dn_tr(I,:)'), 'CB');
            dlTs= dlarray(single(Ts_tr(I,:)'), 'CB');

            [L, grad] = dlfeval(@loss_batch, net, dlX, dly, dlAp, dlBe, dlUp, dlDn, dlTs, lamO, lamU);
            [net, avgGrad, avgSqGrad] = adamupdate(net, grad, avgGrad, avgSqGrad, it, lr);
        end

        if ~isempty(Zva)
            dlXv=dlarray(single(Zva'), 'CB'); dlyv=dlarray(single(yva'), 'CB');
            dlApv=dlarray(single(Ap_va'), 'CB'); dlBev=dlarray(single(Be_va'), 'CB');
            dlUpv=dlarray(single(Up_va'), 'CB'); dlDnv=dlarray(single(Dn_va'), 'CB');
            dlTsv=dlarray(single(Ts_va'), 'CB');
            [Lv,~,MSEv] = dlfeval(@loss_batch, net, dlXv, dlyv, dlApv, dlBev, dlUpv, dlDnv, dlTsv, lamO, lamU);
            fprintf('[epoch %3d/%3d] loss=%.5f | mse=%.5f\n', ep, epochs, double(gather(Lv)), double(gather(MSEv)));
        else
            fprintf('[epoch %3d/%3d] done\n', ep, epochs);
        end
    end

    % —— 保存：把 keepBaseIdx 放入 model.preproc，供控制器使用 —— 关键
    model = struct();
    model.net     = net;
    model.preproc = struct('mu',mu,'sig',sig,'keepBaseIdx',keepBaseIdx);
    save(out_path, 'model','-v7');
    fprintf('[train+gov-soft] 已保存模型：%s\n', out_path);
end

function [L, grad, mse_term] = loss_batch(net, dlX, dly_t, dlAp, dlBe, dlUp, dlDn, dlTs, lamO, lamU)
    dly_hat = forward(net, dlX);  % 1×B
    % teacher 与预测都走一次“简化 governor”，形成软约束
    a_slew_t = min(max(dly_t, dlAp - dlDn.*dlTs), dlAp + dlUp.*dlTs);
    a_star   = dlBe .* dlAp + (1 - dlBe) .* a_slew_t;

    a_slew_h = min(max(dly_hat, dlAp - dlDn.*dlTs), dlAp + dlUp.*dlTs);
    a_post_h = dlBe .* dlAp + (1 - dlBe) .* a_slew_h;

    mse_term = mean((dly_hat - dly_t).^2, 'all');
    over  = max(a_post_h - a_star, 0);
    under = max(a_star   - a_post_h, 0);
    phys  = lamO * mean(over.^2, 'all') + lamU * mean(under.^2, 'all');

    L = mse_term + phys;
    grad = dlgradient(L, net.Learnables);
end

function v = getfield_def(S,fn,def), if isstruct(S)&&isfield(S,fn)&&~isempty(S.(fn)), v=S.(fn); else, v=def; end, end
